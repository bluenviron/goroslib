// Package msgconv contains functions to convert messages from ROS definitions to Golang.
package msgconv

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
	for i := 0; i < len(tmp); i++ {
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

// MessageDefinition is a message definition.
type MessageDefinition struct {
	RosPkgName     string
	Name           string
	Fields         []Field
	Definitions    []Definition
	DefinitionsStr string
	Imports        map[string]struct{}
}

func parseField(goPkgName string, res *MessageDefinition, typ string, name string) error {
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
			return "", "int8 `rostype:\"byte\"`"

		case "char":
			return "", "uint8 `rostype:\"char\"`"
		}

		// implicit package, other message
		return "", f.Type
	}()

	res.Fields = append(res.Fields, f)
	return nil
}

func parseDefinition(goPkgName string, res *MessageDefinition, typ string, name string, val string) error {
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
	return nil
}

// ParseMessageDefinition parses a message definition.
func ParseMessageDefinition(goPkgName string, rosPkgName, name string, content string) (*MessageDefinition, error) {
	res := &MessageDefinition{
		RosPkgName: rosPkgName,
		Name:       strings.Title(name),
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
		typ, line := line[:i], line[i+1:]

		line = strings.TrimLeft(line, " \t")

		i = strings.IndexByte(line, '=')
		if i < 0 {
			name := line
			err := parseField(goPkgName, res, typ, name)
			if err != nil {
				return nil, err
			}

		} else {
			name, val := line[:i], line[i+1:]
			name = strings.TrimRight(name, " \t")
			val = strings.TrimLeft(val, " \t")
			err := parseDefinition(goPkgName, res, typ, name, val)
			if err != nil {
				return nil, err
			}
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
		"github.com/aler9/goroslib/pkg/msg": {},
	}
	for _, f := range res.Fields {
		switch f.TypePkg {
		case "":

		case "time":
			res.Imports["time"] = struct{}{}

		default:
			res.Imports["github.com/aler9/goroslib/pkg/msgs/"+f.TypePkg] = struct{}{}
		}
	}

	return res, nil
}

// Write converts a message definition into a Go structure.
func (res *MessageDefinition) Write() (string, error) {
	buf := bytes.NewBuffer(nil)
	err := tpl.Execute(buf, res)
	if err != nil {
		return "", err
	}

	return buf.String(), nil
}
