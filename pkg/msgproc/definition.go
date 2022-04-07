package msgproc

import (
	"fmt"
	"reflect"
	"strconv"
	"strings"
	"time"

	rmsg "github.com/aler9/goroslib/pkg/msg"
)

type def struct {
	name string
	body string
}

// Definition returns the definition of a message and of all its parents.
func Definition(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	defsByName := make(map[string]struct{})
	var defs []def
	err := definitionMsg(defsByName, &defs, msgt)
	if err != nil {
		return "", err
	}

	// main message
	ret := defs[len(defs)-1].body + "\n"

	// parent messages
	for i := len(defs) - 2; i >= 0; i-- {
		ret += strings.Repeat("=", 80) + "\n"
		ret += "MSG: " + defs[i].name + "\n"
		ret += defs[i].body + "\n"
	}

	return ret, nil
}

func definitionMsg(defsByName map[string]struct{}, defs *[]def, msgt reflect.Type) error {
	body := ""
	nf := msgt.NumField()
	for i := 0; i < nf; i++ {
		ft := msgt.Field(i)

		if ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Package(0)) {
			continue
		}

		if ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Definitions(0)) {
			for _, def := range strings.Split(ft.Tag.Get("ros"), ",") {
				body += def + "\n"
			}
			continue
		}

		name := func() string {
			tagName := ft.Tag.Get("rosname")
			if tagName != "" {
				return tagName
			}
			return camelToSnake(ft.Name)
		}()

		text, err := definitionField(defsByName, defs, ft.Type, ft.Tag.Get("rostype"))
		if err != nil {
			return err
		}

		body += text + " " + name + "\n"
	}

	msgType := typ(msgt)

	if _, ok := defsByName[msgType]; !ok {
		defsByName[msgType] = struct{}{}
		*defs = append(*defs, def{
			name: msgType,
			body: body,
		})
	}

	return nil
}

func definitionField(defsByName map[string]struct{}, defs *[]def, fieldt reflect.Type,
	rosTag string,
) (string, error) {
	switch fieldt {
	case reflect.TypeOf(bool(false)):
		return "bool", nil

	case reflect.TypeOf(int8(0)):
		if rosTag == "byte" {
			return "byte", nil
		}
		return "int8", nil

	case reflect.TypeOf(uint8(0)):
		if rosTag == "char" {
			return "char", nil
		}
		return "uint8", nil

	case reflect.TypeOf(int16(0)):
		return "int16", nil

	case reflect.TypeOf(uint16(0)):
		return "uint16", nil

	case reflect.TypeOf(int32(0)):
		return "int32", nil

	case reflect.TypeOf(uint32(0)):
		return "uint32", nil

	case reflect.TypeOf(int64(0)):
		return "int64", nil

	case reflect.TypeOf(uint64(0)):
		return "uint64", nil

	case reflect.TypeOf(float32(0)):
		return "float32", nil

	case reflect.TypeOf(float64(0)):
		return "float64", nil

	case reflect.TypeOf(string("")):
		return "string", nil

	case reflect.TypeOf(time.Time{}):
		return "time", nil

	case reflect.TypeOf(time.Duration(0)):
		return "duration", nil
	}

	switch fieldt.Kind() {
	case reflect.Slice:
		def, err := definitionField(defsByName, defs, fieldt.Elem(), "")
		if err != nil {
			return "", err
		}

		return def + "[]", nil

	case reflect.Array:
		def, err := definitionField(defsByName, defs, fieldt.Elem(), "")
		if err != nil {
			return "", err
		}

		return def + "[" + strconv.FormatInt(int64(fieldt.Len()), 10) + "]", nil

	case reflect.Struct:
		err := definitionMsg(defsByName, defs, fieldt)
		if err != nil {
			return "", err
		}

		return typ(fieldt), nil
	}

	return "", fmt.Errorf("unsupported field type '%s'", fieldt.String())
}
