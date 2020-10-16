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

var tpl = template.Must(template.New("").Parse(
	`package {{ .GoPkgName }}

import (
{{- range $k, $v := .Imports }}
    "{{ $k }}"
{{- end }}
)
{{- if .Definitions }}
{{- $MsgName := .Name }}

const (
{{- range .Definitions }}
    {{ $MsgName }}_{{ .Name }} {{ .Type }} = {{ .Value }}
{{- end }}
)
{{- end }}

type {{ .Name }} struct {
    msg.Package ` + "`" + `ros:"{{ .RosPkgName }}"` + "`" + `
{{- if .DefinitionsStr }}
    msg.Definitions ` + "`" + `ros:"{{ .DefinitionsStr }}"` + "`" + `
{{- end }}
{{- range .Fields }}
{{- if .TypePkg }}
    {{ .Name }} {{ .TypeArray }}{{ .TypePkg }}.{{ .Type }}
{{- else }}
    {{ .Name }} {{ .TypeArray }}{{ .Type }}
{{- end }}
{{- end }}
}
`))

type Definition struct {
	Type  string
	Name  string
	Value string
}

type Field struct {
	TypePkg   string
	TypeArray string
	Type      string
	Name      string
}

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

func camelToSnake(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToLower(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if unicode.IsUpper(tmp[i]) {
			tmp[i] = unicode.ToLower(tmp[i])
			tmp = append(tmp[:i], append([]rune{'_'}, tmp[i:]...)...)
		}
	}
	return string(tmp)
}

func download(addr string) ([]byte, error) {
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
	argUrl := kingpin.Arg("urls", "paths or urls pointing to ROS messages").Required().String()

	kingpin.Parse()

	goPkgName := *argGoPkgName
	rosPkgName := *argRosPkgName
	u := *argUrl

	isRemote := func() bool {
		_, err := url.ParseRequestURI(u)
		return err == nil
	}()

	content, err := func() (string, error) {
		if isRemote {
			byts, err := download(u)
			if err != nil {
				return "", err
			}
			return string(byts), nil
		}

		byts, err := ioutil.ReadFile(u)
		if err != nil {
			return "", err
		}
		return string(byts), nil
	}()
	if err != nil {
		return err
	}

	var fields []Field
	var definitions []Definition

	name := func() string {
		if isRemote {
			ur, _ := url.Parse(u)
			return strings.TrimSuffix(filepath.Base(ur.Path), ".msg")
		}
		return strings.TrimSuffix(filepath.Base(u), ".msg")
	}()

	for _, line := range strings.Split(content, "\n") {
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

			d := Definition{
				Type:  matches[1],
				Name:  matches[3],
				Value: matches[6],
			}

			switch d.Type {
			case "byte":
				d.Type = "int8"

			case "char":
				d.Type = "uint8"
			}

			definitions = append(definitions, d)

			// field
		} else {
			// remove multiple spaces between type and name
			line = regexp.MustCompile("\\s+").ReplaceAllString(line, " ")

			parts := strings.Split(line, " ")
			if len(parts) != 2 {
				return fmt.Errorf("line does not contain 2 fields")
			}

			f := Field{}

			f.Name = snakeToCamel(parts[1])

			if camelToSnake(f.Name) != parts[1] {
				return fmt.Errorf("The field `%s` can't be used, since is not in snake case.", f.Name)
			}

			f.Type = parts[0]

			// split TypeArray and Type
			ma := regexp.MustCompile("^(.+?)(\\[.*?\\])$").FindStringSubmatch(f.Type)
			if ma != nil {
				f.TypeArray = ma[2]
				f.Type = ma[1]
			}

			f.TypePkg, f.Type = func() (string, string) {
				// explicit package
				parts := strings.Split(f.Type, "/")
				if len(parts) == 2 {
					// type of same package
					if parts[0] == goPkgName {
						return "", parts[1]
					}

					// type of other package
					return parts[0], parts[1]
				}

				// implicit package, type of std_msgs
				if goPkgName != "std_msgs" {
					switch f.Type {
					case "Bool", "ColorRGBA",
						"Duration", "Empty", "Float32MultiArray", "Float32",
						"Float64MultiArray", "Float64", "Header", "Int8MultiArray",
						"Int8", "Int16MultiArray", "Int16", "Int32MultiArray", "Int32",
						"Int64MultiArray", "Int64", "MultiArrayDimension", "MultiarrayLayout",
						"String", "Time", "UInt8MultiArray", "UInt8", "UInt16MultiArray", "UInt16",
						"UInt32MultiArray", "UInt32", "UInt64MultiArray", "UInt64":
						return "std_msgs", parts[0]
					}
				}

				// implicit package, native type
				switch f.Type {
				case "bool", "int8", "uint8", "int16", "uint16",
					"int32", "uint32", "int64", "uint64", "float32",
					"float64", "string":
					return "", f.Type

				case "time", "duration":
					return "time", strings.Title(f.Type)

				case "byte":
					return "", "int8 `ros:\"byte\"`"

				case "char":
					return "", "uint8 `ros:\"char\"`"
				}

				// implicit package, other message
				return "", f.Type
			}()

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

	imports := map[string]struct{}{
		"github.com/aler9/goroslib/msg": {},
	}
	for _, f := range fields {
		switch f.TypePkg {
		case "":

		case "time":
			imports["time"] = struct{}{}

		default:
			imports["github.com/aler9/goroslib/msgs/"+f.TypePkg] = struct{}{}
		}
	}

	return tpl.Execute(os.Stdout, map[string]interface{}{
		"GoPkgName":      goPkgName,
		"RosPkgName":     rosPkgName,
		"Name":           name,
		"Fields":         fields,
		"Definitions":    definitions,
		"DefinitionsStr": definitionsStr,
		"Imports":        imports,
	})
}

func main() {
	err := run()
	if err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
