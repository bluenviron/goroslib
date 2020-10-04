package main

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"text/template"
	"unicode"

	"gopkg.in/alecthomas/kingpin.v2"
)

// TODO: readd definitions
/*
{{- if .Definitions }}
const (
{{- range .Definitions }}
{{ .Name }} = {{ .Value }}
{{- end }}
)
{{- end }}
*/

var tpl = template.Must(template.New("").Parse(
	`package {{ .GoPkgName }}

import (
{{- range $k, $v := .Imports }}
    "{{ $k }}"
{{- end }}
)

type {{ .Name }} struct {
    msg.Package ` + "`" + `ros:"{{ .RosPkgName }}"` + "`" + `
{{- if .DefinitionsStr }}
    msg.Definitions ` + "`" + `ros:"{{ .DefinitionsStr }}"` + "`" + `
{{- end }}
{{- range .Fields }}
    {{ .Name }} {{ .Type }}
{{- end }}
}
`))

func snakeToCamel(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if tmp[i] == '_' {
			tmp[i+1] = unicode.ToUpper(tmp[i+1])
			tmp = append(tmp[:i], tmp[i+1:]...)
			i -= 1
		}
	}
	return string(tmp)
}

func downloadBytes(addr string) ([]byte, error) {
	req, err := http.NewRequest("GET", addr, nil)
	if err != nil {
		return nil, err
	}

	res, err := http.DefaultClient.Do(req)
	if err != nil {
		return nil, err
	}
	defer res.Body.Close()

	return ioutil.ReadAll(res.Body)
}

func run() error {
	kingpin.CommandLine.Help = "Convert ROS messages into Go structs."

	argGoPkgName := kingpin.Flag("gopackage", "Go package name").Default("main").String()
	argRosPkgName := kingpin.Flag("rospackage", "ROS package name").Default("my_package").String()
	argDef := kingpin.Arg("def", "a path or url pointing to a ROS message").Required().String()

	kingpin.Parse()

	goPkgName := *argGoPkgName
	rosPkgName := *argRosPkgName
	def := *argDef

	isRemote := func() bool {
		_, err := url.ParseRequestURI(def)
		return err == nil
	}()

	byts, err := func() ([]byte, error) {
		if isRemote {
			return downloadBytes(def)
		}
		return ioutil.ReadFile(def)
	}()
	if err != nil {
		return err
	}

	name := func() string {
		if isRemote {
			ur, _ := url.Parse(def)
			return strings.TrimSuffix(filepath.Base(ur.Path), ".msg")
		}
		return strings.TrimSuffix(filepath.Base(def), ".msg")
	}()

	str := string(byts)
	imports := map[string]struct{}{
		"github.com/aler9/goroslib/msg": {},
	}

	type definition struct {
		Type  string
		Name  string
		Value string
	}
	var definitions []*definition

	type field struct {
		Type string
		Name string
	}
	var fields []*field

	for _, line := range strings.Split(str, "\n") {
		// remove comments
		line = regexp.MustCompile("#.*$").ReplaceAllString(line, "")

		// remove leading and trailing spaces
		line = strings.TrimSpace(line)

		// do not process empty lines
		if line == "" {
			continue
		}

		// definition
		if strings.Contains(line, "=") {
			matches := regexp.MustCompile("^([a-z0-9]+)(\\s|\\t)+([A-Z0-9_]+)(\\s|\\t)*=(\\s|\\t)*(.+?)$").FindStringSubmatch(line)
			if matches == nil {
				return fmt.Errorf("line '%s' is not a definition", line)
			}

			typ, name, val := matches[1], matches[3], matches[6]
			definitions = append(definitions, &definition{typ, name, val})

			// field
		} else {
			// remove multiple spaces between type and name
			line = regexp.MustCompile("\\s+").ReplaceAllString(line, " ")

			parts := strings.Split(line, " ")
			if len(parts) != 2 {
				return fmt.Errorf("line does not contain 2 fields")
			}

			f := &field{}

			f.Name = snakeToCamel(parts[1])

			f.Type = parts[0]
			arrayPrefix := ""
			ma := regexp.MustCompile("^(.+?)(\\[.*?\\])$").FindStringSubmatch(f.Type)
			if ma != nil {
				arrayPrefix = ma[2]
				f.Type = ma[1]
			}

			f.Type = func() string {
				// explicit package
				parts := strings.Split(f.Type, "/")
				if len(parts) == 2 {
					// same package
					if parts[0] == goPkgName {
						return parts[1]
					}

					// other package
					imports["github.com/aler9/goroslib/msgs/"+parts[0]] = struct{}{}
					return parts[0] + "." + parts[1]

					// implicit package
				} else {
					// implicit std_msgs fields
					if goPkgName != "std_msgs" {
						switch f.Type {
						case "Bool", "ColorRGBA",
							"Duration", "Empty", "Float32MultiArray", "Float32",
							"Float64MultiArray", "Float64", "Header", "Int8MultiArray",
							"Int8", "Int16MultiArray", "Int16", "Int32MultiArray", "Int32",
							"Int64MultiArray", "Int64", "MultiArrayDimension", "MultiarrayLayout",
							"String", "Time", "UInt8MultiArray", "UInt8", "UInt16MultiArray", "UInt16",
							"UInt32MultiArray", "UInt32", "UInt64MultiArray", "UInt64":
							imports["github.com/aler9/goroslib/msgs/std_msgs"] = struct{}{}
							return "std_msgs." + parts[0]
						}
					}
				}

				// native types
				switch f.Type {
				case "bool", "int8", "uint8", "int16", "uint16",
					"int32", "uint32", "int64", "uint64", "float32",
					"float64", "string":
					return f.Type

				case "time", "duration":
					imports["time"] = struct{}{}
					return "time." + strings.Title(f.Type)

				case "byte":
					return "int8 `ros:\"byte\"`"

				case "char":
					return "uint8 `ros:\"char\"`"
				}

				return f.Type
			}()

			if arrayPrefix != "" {
				f.Type = arrayPrefix + f.Type
			}

			fields = append(fields, f)
		}
	}

	definitionsStr := func() string {
		var tmp []string
		for _, d := range definitions {
			tmp = append(tmp, d.Type+" "+d.Name+"="+d.Value)
		}
		return strings.Join(tmp, ",")
	}()

	return tpl.Execute(os.Stdout, map[string]interface{}{
		"GoPkgName":      goPkgName,
		"RosPkgName":     rosPkgName,
		"Imports":        imports,
		"Name":           name,
		"DefinitionsStr": definitionsStr,
		"Definitions":    definitions,
		"Fields":         fields,
	})
}

func main() {
	err := run()
	if err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
