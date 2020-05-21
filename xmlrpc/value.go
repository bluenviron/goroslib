package xmlrpc

import (
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
	"strconv"
)

func decodeBool(in []byte, val reflect.Value) error {
	if len(in) != 1 {
		return fmt.Errorf("value is not a bool: %v", in)
	}

	var v bool
	switch in[0] {
	case '1':
		v = true

	case '0':
		v = false

	default:
		return fmt.Errorf("value is not a bool: %v", in)
	}

	switch val.Elem().Kind() {
	case reflect.Bool:
		*(val.Interface().(*bool)) = v

	default:
		return fmt.Errorf("cannot decode a bool into a %s", val.Elem().Kind())
	}
	return nil
}

func decodeInt(in []byte, val reflect.Value) error {
	// in xmlrpc numbers are always 32 bits
	v, err := strconv.ParseInt(string(in), 10, 32)
	if err != nil {
		return err
	}

	switch val.Elem().Kind() {
	case reflect.Int:
		*(val.Interface().(*int)) = int(v)

	default:
		return fmt.Errorf("cannot decode a int into a %s", val.Elem().Kind())
	}
	return nil
}

func decodeDouble(in []byte, val reflect.Value) error {
	v, err := strconv.ParseFloat(string(in), 64)
	if err != nil {
		return err
	}

	switch val.Elem().Kind() {
	case reflect.Float64:
		*(val.Interface().(*float64)) = v

	default:
		return fmt.Errorf("cannot decode a double into a %s", val.Elem().Kind())
	}
	return nil
}

func decodeString(in []byte, val reflect.Value) error {
	switch val.Elem().Kind() {
	case reflect.String:
		*(val.Interface().(*string)) = string(in)

	default:
		return fmt.Errorf("cannot decode a string into a %s", val.Elem().Kind())
	}
	return nil
}

func decodeArray(dec *xml.Decoder, val reflect.Value) error {
	err := xmlGetStartElement(dec, "data")
	if err != nil {
		return err
	}

	switch val.Elem().Kind() {
	case reflect.Struct:
		nf := val.Elem().NumField()
		for i := 0; i < nf; i++ {
			field := val.Elem().Field(i).Addr()

			err := xmlGetStartElement(dec, "value")
			if err != nil {
				return err
			}

			err = decodeValue(dec, field)
			if err != nil {
				return err
			}
		}

		err = xmlGetEndElement(dec, "data")
		if err != nil {
			return err
		}

	case reflect.Slice:
		for {
			err := xmlGetStartElement(dec, "value")
			if err != nil {
				// slice is over
				if err == errEndElement {
					break
				}
				return err
			}

			el := reflect.New(val.Elem().Type().Elem())
			err = decodeValue(dec, el)
			if err != nil {
				return err
			}

			val.Elem().Set(reflect.Append(val.Elem(), el.Elem()))
		}

	default:
		return fmt.Errorf("cannot decode an array into a %s", val.Elem().Kind())
	}

	return xmlGetEndElement(dec, "array")
}

func decodeValue(dec *xml.Decoder, val reflect.Value) error {
	tok, err := dec.Token()
	if err != nil {
		return err
	}

	switch ttok := tok.(type) {
	// type name
	case xml.StartElement:
		switch ttok.Name.Local {
		case "boolean":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeBool(cnt, val)
			if err != nil {
				return err
			}

		case "int", "i4":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeInt(cnt, val)
			if err != nil {
				return err
			}

		case "double":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeDouble(cnt, val)
			if err != nil {
				return err
			}

		case "string":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeString(cnt, val)
			if err != nil {
				return err
			}

		case "array":
			err = decodeArray(dec, val)
			if err != nil {
				return err
			}

		default:
			return fmt.Errorf("unhandled value type: %s", ttok.Name.Local)
		}

		err = xmlGetEndElement(dec, "value")
		if err != nil {
			return err
		}

	// content with no type (i.e. string)
	case xml.CharData:
		err := decodeString(ttok, val)
		if err != nil {
			return err
		}

		err = xmlGetEndElement(dec, "value")
		if err != nil {
			return err
		}

	// content with no type (i.e. string) and empty
	case xml.EndElement:
		err := decodeString([]byte{}, val)
		if err != nil {
			return err
		}

	default:
		return fmt.Errorf("unexpected element type: %T", tok)
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
