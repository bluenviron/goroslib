package msg

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"
	"strconv"
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

func md5Text(rt reflect.Type) (string, bool, error) {
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}

	switch rt {
	case reflect.TypeOf(Bool(false)):
		return "bool", false, nil

	case reflect.TypeOf(Byte(0)):
		return "byte", false, nil

	case reflect.TypeOf(Char(0)):
		return "char", false, nil

	case reflect.TypeOf(Int8(0)):
		return "int8", false, nil

	case reflect.TypeOf(Uint8(0)):
		return "uint8", false, nil

	case reflect.TypeOf(Int16(0)):
		return "int16", false, nil

	case reflect.TypeOf(Uint16(0)):
		return "uint16", false, nil

	case reflect.TypeOf(Int32(0)):
		return "int32", false, nil

	case reflect.TypeOf(Uint32(0)):
		return "uint32", false, nil

	case reflect.TypeOf(Int64(0)):
		return "int64", false, nil

	case reflect.TypeOf(Uint64(0)):
		return "uint64", false, nil

	case reflect.TypeOf(Float32(0)):
		return "float32", false, nil

	case reflect.TypeOf(Float64(0)):
		return "float64", false, nil

	case reflect.TypeOf(String(0)):
		return "string", false, nil

	case reflect.TypeOf(Time{}):
		return "time", false, nil

	case reflect.TypeOf(Duration(0)):
		return "duration", false, nil
	}

	switch rt.Kind() {
	case reflect.Slice:
		text, isstruct, err := md5Text(rt.Elem())
		if err != nil {
			return "", false, err
		}

		if isstruct {
			return text, true, nil
		}

		return text + "[]", false, nil

	case reflect.Array:
		text, isstruct, err := md5Text(rt.Elem())
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
			name := camelToSnake(rt.Field(i).Name)

			text, isstruct, err := md5Text(rt.Field(i).Type)
			if err != nil {
				return "", false, err
			}

			if isstruct {
				text = md5Sum(text)
			}

			ret += text
			ret += " "
			ret += name
			if (i + 1) != nf {
				ret += "\n"
			}
		}
		return ret, true, nil
	}

	return "", false, fmt.Errorf("unsupported type '%s'", rt.String())
}

func MessageMd5(msg interface{}) (string, error) {
	text, _, err := md5Text(reflect.TypeOf(msg))
	if err != nil {
		return "", err
	}
	return md5Sum(text), nil
}

func ServiceMd5(req interface{}, res interface{}) (string, error) {
	text1, _, err := md5Text(reflect.TypeOf(req))
	if err != nil {
		return "", err
	}

	text2, _, err := md5Text(reflect.TypeOf(res))
	if err != nil {
		return "", err
	}

	return md5Sum(text1 + text2), nil
}
