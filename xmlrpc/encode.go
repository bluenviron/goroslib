package xmlrpc

import (
	"fmt"
	"io"
	"reflect"
	"strconv"
)

func requestEncode(w io.Writer, method string, params interface{}) error {
	_, err := w.Write([]byte(`<?xml version="1.0"?><methodCall>` +
		`<methodName>` + method + `</methodName><params>`))
	if err != nil {
		return err
	}

	// write each param
	rv := reflect.ValueOf(params)
	nf := rv.NumField()
	for i := 0; i < nf; i++ {
		field := rv.Field(i)

		_, err := w.Write([]byte(`<param>`))
		if err != nil {
			return err
		}

		err = encodeValue(w, field)
		if err != nil {
			return err
		}

		_, err = w.Write([]byte(`</param>`))
		if err != nil {
			return err
		}
	}

	_, err = w.Write([]byte(`</params></methodCall>`))
	if err != nil {
		return err
	}

	return nil
}

func responseEncode(w io.Writer, params interface{}) error {
	_, err := w.Write([]byte(`<?xml version="1.0"?><methodResponse><params>` +
		`<param><value><array><data>`))
	if err != nil {
		return err
	}

	// write each param
	rv := reflect.ValueOf(params)
	nf := rv.NumField()
	for i := 0; i < nf; i++ {
		field := rv.Field(i)

		err = encodeValue(w, field)
		if err != nil {
			return err
		}
	}

	_, err = w.Write([]byte(`</data></array></value></param>` +
		`</params></methodResponse>`))
	if err != nil {
		return err
	}

	return nil
}

func encodeValue(w io.Writer, val reflect.Value) error {
	_, err := w.Write([]byte(`<value>`))
	if err != nil {
		return err
	}

	switch val.Kind() {
	case reflect.Bool:
		v := "0"
		if val.Interface().(bool) {
			v = "1"
		}
		_, err := w.Write([]byte(`<boolean>` + v + `</boolean>`))
		if err != nil {
			return err
		}

	case reflect.Int:
		_, err := w.Write([]byte(`<i4>` + strconv.FormatInt(int64(val.Interface().(int)), 10) + `</i4>`))
		if err != nil {
			return err
		}

	case reflect.Float64:
		_, err := w.Write([]byte(`<double>` + strconv.FormatFloat(val.Interface().(float64), 'G', -1, 64) + `</double>`))
		if err != nil {
			return err
		}

	case reflect.String:
		_, err := w.Write([]byte(val.Interface().(string)))
		if err != nil {
			return err
		}

	case reflect.Struct:
		_, err := w.Write([]byte(`<array><data>`))
		if err != nil {
			return err
		}

		nf := val.NumField()
		for i := 0; i < nf; i++ {
			field := val.Field(i)

			err := encodeValue(w, field)
			if err != nil {
				return err
			}
		}

		_, err = w.Write([]byte(`</data></array>`))
		if err != nil {
			return err
		}

	case reflect.Slice:
		_, err := w.Write([]byte(`<array><data>`))
		if err != nil {
			return err
		}

		le := val.Len()
		for i := 0; i < le; i++ {
			el := val.Index(i)

			err := encodeValue(w, el)
			if err != nil {
				return err
			}
		}

		_, err = w.Write([]byte(`</data></array>`))
		if err != nil {
			return err
		}

	default:
		return fmt.Errorf("unhandled value type: %s", val.Kind())
	}

	_, err = w.Write([]byte(`</value>`))
	if err != nil {
		return err
	}

	return nil
}
