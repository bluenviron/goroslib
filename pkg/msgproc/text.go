package msgproc

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"
	"strconv"
	"strings"
	"time"
	"unicode"

	rmsg "github.com/bluenviron/goroslib/v2/pkg/msg"
)

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

func md5sum(text string) string {
	h := md5.New()
	h.Write([]byte(text))
	return hex.EncodeToString(h.Sum(nil))
}

// Text returns the text equivalent of a message.
func Text(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	return textMsg(msgt)
}

func textMsg(msgt reflect.Type) (string, error) {
	ret := ""
	nf := msgt.NumField()
	for i := 0; i < nf; i++ {
		ft := msgt.Field(i)

		if ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Package(0)) {
			continue
		}

		if ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Name(0)) {
			continue
		}

		if ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Definitions(0)) {
			for _, def := range strings.Split(ft.Tag.Get("ros"), ",") {
				ret += def + "\n"
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

		text, _, err := textField(ft.Type, ft.Tag.Get("rostype"))
		if err != nil {
			return "", err
		}

		ret += text + " " + name + "\n"
	}

	// Remove trailing newline
	if len(ret) > 0 {
		ret = ret[:len(ret)-1]
	}

	return ret, nil
}

func textField(fieldt reflect.Type, rosTag string) (string, bool, error) {
	switch fieldt {
	case reflect.TypeOf(bool(false)):
		return "bool", false, nil

	case reflect.TypeOf(int8(0)):
		if rosTag == "byte" {
			return "byte", false, nil
		}
		return "int8", false, nil

	case reflect.TypeOf(uint8(0)):
		if rosTag == "char" {
			return "char", false, nil
		}
		return "uint8", false, nil

	case reflect.TypeOf(int16(0)):
		return "int16", false, nil

	case reflect.TypeOf(uint16(0)):
		return "uint16", false, nil

	case reflect.TypeOf(int32(0)):
		return "int32", false, nil

	case reflect.TypeOf(uint32(0)):
		return "uint32", false, nil

	case reflect.TypeOf(int64(0)):
		return "int64", false, nil

	case reflect.TypeOf(uint64(0)):
		return "uint64", false, nil

	case reflect.TypeOf(float32(0)):
		return "float32", false, nil

	case reflect.TypeOf(float64(0)):
		return "float64", false, nil

	case reflect.TypeOf(string("")):
		return "string", false, nil

	case reflect.TypeOf(time.Time{}):
		return "time", false, nil

	case reflect.TypeOf(time.Duration(0)):
		return "duration", false, nil
	}

	switch fieldt.Kind() {
	case reflect.Slice:
		text, isStruct, err := textField(fieldt.Elem(), "")
		if err != nil {
			return "", false, err
		}

		if isStruct {
			return text, false, nil
		}

		return text + "[]", false, nil

	case reflect.Array:
		text, isStruct, err := textField(fieldt.Elem(), "")
		if err != nil {
			return "", false, err
		}

		if isStruct {
			return text, false, nil
		}

		return text + "[" + strconv.FormatInt(int64(fieldt.Len()), 10) + "]", false, nil

	case reflect.Struct:
		text, err := textMsg(fieldt)
		if err != nil {
			return "", false, err
		}

		return md5sum(text), true, nil
	}

	return "", false, fmt.Errorf("unsupported field type '%s'", fieldt.String())
}
