package conversion

import (
	"bytes"
	"fmt"
	"regexp"
	"strings"
	"text/template"
	"unicode"
)

var tpl = template.Must(template.New("").Parse(
	`{{- if .Definitions }}
{{- $MsgName := .Name }}

const (
{{- range .Definitions }}
{{- if eq .GoType "string" }}
    {{ $MsgName }}_{{ .Name }} {{ .GoType }} = "{{ .Value }}"
{{- else }}
    {{ $MsgName }}_{{ .Name }} {{ .GoType }} = {{ .Value }}
{{- end }}
{{- end }}
)
{{- end }}

type {{ .Name }} struct {
{{- if .RosPkgName }}
    msg.Package ` + "`" + `ros:"{{ .RosPkgName }}"` + "`" + `
{{- end }}
{{- if .DefinitionsStr }}
    msg.Definitions ` + "`" + `ros:"{{ .DefinitionsStr }}"` + "`" + `
{{- end }}
{{- range .Fields }}
    {{ .Name }} ` +
		`{{ if .TypePkg -}}{{ .TypeArray }}{{ .TypePkg }}.{{ .Type }}{{- else -}}{{ .TypeArray }}{{ .Type }}{{- end -}}` +
		`{{ if .NameOverride -}} ` + "`" + `rosname:"{{ .NameOverride }}"` + "`" + `{{- end -}}
{{- end }}
}
`))

func firstCharToUpper(s string) string {
	return strings.ToUpper(s[:1]) + s[1:]
}

// Definition is a message definition.
type Definition struct {
	RosType   string
	GoType    string
	Name      string
	Value     string
	HasQuotes bool
}

// Field is a message field.
type Field struct {
	TypeArray    string
	TypePkg      string
	Type         string
	Name         string
	NameOverride string
}

func snakeToCamel(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < (len(tmp) - 1); i++ {
		if tmp[i] == '_' {
			tmp[i+1] = unicode.ToUpper(tmp[i+1])
			tmp = append(tmp[:i], tmp[i+1:]...)
			i--
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

type messageDefinition struct {
	RosPkgName     string
	Name           string
	Fields         []Field
	Definitions    []Definition
	DefinitionsStr string
	Imports        map[string]struct{}
}

func parseField(rosPkgName string, res *messageDefinition, typ string, name string) {
	f := Field{}

	// use NameOverride if a bidirectional conversion between snake and
	// camel is not possible
	f.Name = snakeToCamel(name)
	if camelToSnake(f.Name) != name {
		f.NameOverride = name
	}

	// split TypeArray and Type
	ma := regexp.MustCompile(`^(.+?)(\[.*?\])$`).FindStringSubmatch(typ)
	if ma != nil {
		f.TypeArray = ma[2]
		f.Type = ma[1]
	} else {
		f.Type = typ
	}

	f.TypePkg, f.Type = func() (string, string) {
		// explicit package
		parts := strings.Split(f.Type, "/")
		if len(parts) == 2 {
			// type of same package
			if parts[0] == rosPkgName {
				return "", parts[1]
			}

			// type of other package
			return parts[0], parts[1]
		}

		// implicit package, type of std_msgs
		if rosPkgName != "std_msgs" {
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
			return "time", firstCharToUpper(f.Type)

		case "byte":
			return "", "int8 `rostype:\"byte\"`"

		case "char":
			return "", "uint8 `rostype:\"char\"`"
		}

		// implicit package, other message
		return "", f.Type
	}()

	res.Fields = append(res.Fields, f)
}

func parseDefinition(res *messageDefinition, typ string, name string, val string) {
	d := Definition{
		RosType: typ,
		Name:    name,
		Value:   val,
	}

	d.Value = strings.ReplaceAll(d.Value, "\"", "\\\"")

	d.GoType = func() string {
		switch d.RosType {
		case "byte":
			return "int8"

		case "char":
			return "uint8"
		}
		return d.RosType
	}()

	res.Definitions = append(res.Definitions, d)
}

func parseMessageDefinition(rosPkgName string, name string, content string) (*messageDefinition, error) {
	res := &messageDefinition{
		RosPkgName: rosPkgName,
		Name:       firstCharToUpper(name),
	}

	for _, line := range strings.Split(content, "\n") {
		// remove comments
		line = regexp.MustCompile("#.*$").ReplaceAllString(line, "")

		// remove leading and trailing spaces
		line = strings.TrimSpace(line)

		// skip empty lines
		if line == "" {
			continue
		}

		i := strings.IndexAny(line, " \t")
		if i < 0 {
			return nil, fmt.Errorf("unable to parse line (%s)", line)
		}

		var typ string
		typ, line = line[:i], line[i+1:]

		line = strings.TrimLeft(line, " \t")

		i = strings.IndexByte(line, '=')
		if i < 0 {
			name := line
			parseField(rosPkgName, res, typ, name)
		} else {
			name, val := line[:i], line[i+1:]
			name = strings.TrimRight(name, " \t")
			val = strings.TrimLeft(val, " \t")
			parseDefinition(res, typ, name, val)
		}
	}

	res.DefinitionsStr = func() string {
		var tmp []string
		for _, d := range res.Definitions {
			tmp = append(tmp, d.RosType+" "+d.Name+"="+d.Value)
		}
		return strings.Join(tmp, ",")
	}()

	res.Imports = map[string]struct{}{
		"github.com/bluenviron/goroslib/v2/pkg/msg": {},
	}
	for _, f := range res.Fields {
		switch f.TypePkg {
		case "":

		case "time":
			res.Imports["time"] = struct{}{}

		default:
			res.Imports["github.com/bluenviron/goroslib/v2/pkg/msgs/"+f.TypePkg] = struct{}{}
		}
	}

	return res, nil
}

func (res *messageDefinition) write() (string, error) {
	var buf bytes.Buffer
	err := tpl.Execute(&buf, res)
	if err != nil {
		return "", err
	}
	return buf.String(), nil
}
