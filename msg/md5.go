package msg

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"
	"strconv"
	"strings"
	"time"
	"unicode"
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

func md5Sum(text string) string {
	h := md5.New()
	h.Write([]byte(text))
	return hex.EncodeToString(h.Sum(nil))
}

func md5Text(rt reflect.Type, rosTag string) (string, bool, error) {
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
		text, isstruct, err := md5Text(rt.Elem(), "")
		if err != nil {
			return "", false, err
		}

		if isstruct {
			return text, true, nil
		}

		return text + "[]", false, nil

	case reflect.Array:
		text, isstruct, err := md5Text(rt.Elem(), "")
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

			if ft.Anonymous && ft.Type == reflect.TypeOf(Package(0)) {
				continue
			}

			if ft.Anonymous && ft.Type == reflect.TypeOf(Definitions(0)) {
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

			text, isstruct, err := md5Text(ft.Type, ft.Tag.Get("rostype"))
			if err != nil {
				return "", false, err
			}

			if isstruct {
				text = md5Sum(text)
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

// Md5Message computes the checksum of a message.
func Md5Message(msg interface{}) (string, error) {
	rt := reflect.TypeOf(msg)
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", rt.String())
	}

	text, _, err := md5Text(rt, "")
	if err != nil {
		return "", err
	}

	return md5Sum(text), nil
}

// Md5Message computes the checksum of a service.
func Md5Service(req interface{}, res interface{}) (string, error) {
	reqt := reflect.TypeOf(req)
	if reqt.Kind() == reflect.Ptr {
		reqt = reqt.Elem()
	}
	if reqt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", reqt.String())
	}

	text1, _, err := md5Text(reqt, "")
	if err != nil {
		return "", err
	}

	rest := reflect.TypeOf(res)
	if rest.Kind() == reflect.Ptr {
		rest = rest.Elem()
	}
	if rest.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", rest.String())
	}

	text2, _, err := md5Text(rest, "")
	if err != nil {
		return "", err
	}

	return md5Sum(text1 + text2), nil
}
