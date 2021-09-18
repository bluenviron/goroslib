// Package msgproc contains functions to process messages.
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

	"github.com/aler9/goroslib/pkg/msg"
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

// Text processes a field or message and returns its equivalent in text format.
func Text(rt reflect.Type, rosTag string) (string, bool, error) {
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}

	switch rt {
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

	switch rt.Kind() {
	case reflect.Slice:
		text, isstruct, err := Text(rt.Elem(), "")
		if err != nil {
			return "", false, err
		}

		if isstruct {
			return text, true, nil
		}

		return text + "[]", false, nil

	case reflect.Array:
		text, isstruct, err := Text(rt.Elem(), "")
		if err != nil {
			return "", false, err
		}

		if isstruct {
			return text, true, nil
		}

		return text + "[" + strconv.FormatInt(int64(rt.Len()), 10) + "]", false, nil

	case reflect.Struct:
		ret := ""
		nf := rt.NumField()
		for i := 0; i < nf; i++ {
			ft := rt.Field(i)

			if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Package(0)) {
				continue
			}

			if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Definitions(0)) {
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

			text, isstruct, err := Text(ft.Type, ft.Tag.Get("rostype"))
			if err != nil {
				return "", false, err
			}

			if isstruct {
				text = md5sum(text)
			}

			ret += text + " " + name + "\n"
		}

		// Remove trailing newline
		if len(ret) > 0 {
			ret = ret[:len(ret)-1]
		}

		return ret, true, nil
	}

	return "", false, fmt.Errorf("unsupported field type '%s'", rt.String())
}

// MD5 computes the checksum of a message.
func MD5(msg interface{}) (string, error) {
	rt := reflect.TypeOf(msg)
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", rt.String())
	}

	text, _, err := Text(rt, "")
	if err != nil {
		return "", err
	}

	return md5sum(text), nil
}

// Type returns the type of a message.
func Type(m interface{}) (string, error) {
	rt := reflect.TypeOf(m)
	if rt.Kind() != reflect.Ptr {
		return "", fmt.Errorf("message must be a pointer")
	}
	rt = rt.Elem()
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a pointer to a struct")
	}

	name := rt.Name()
	if name == "" {
		name = "Msg"
	}

	pkg := func() string {
		ft, ok := rt.FieldByName("Package")
		if !ok || !ft.Anonymous || ft.Type != reflect.TypeOf(msg.Package(0)) {
			return ""
		}

		return ft.Tag.Get("ros")
	}()
	if pkg == "" {
		pkg = "goroslib"
	}

	return pkg + "/" + name, nil
}
