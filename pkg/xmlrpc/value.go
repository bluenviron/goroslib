package xmlrpc

import (
	"encoding/base64"
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
	"strconv"
)

func decodeBool(in []byte, val reflect.Value) error {
	if len(in) != 1 {
		return fmt.Errorf("value is not a bool: '%v'", string(in))
	}

	var v bool
	switch in[0] {
	case '1':
		v = true

	case '0':
		v = false

	default:
		return fmt.Errorf("value is not a bool: '%v'", string(in))
	}

	switch tval := val.Interface().(type) {
	case *bool:
		*tval = v

	case *interface{}:
		*tval = v

	default:
		return fmt.Errorf("cannot decode a bool into a %T", val.Elem().Interface())
	}

	return nil
}

func decodeInt(in []byte, val reflect.Value) error {
	// in xmlrpc numbers are always 32 bits
	v, err := strconv.ParseInt(string(in), 10, 32)
	if err != nil {
		return err
	}

	switch tval := val.Interface().(type) {
	case *int:
		*tval = int(v)

	case *interface{}:
		*tval = int(v)

	default:
		return fmt.Errorf("cannot decode a int into a %T", val.Elem().Interface())
	}

	return nil
}

func decodeDouble(in []byte, val reflect.Value) error {
	v, err := strconv.ParseFloat(string(in), 64)
	if err != nil {
		return err
	}

	switch tval := val.Interface().(type) {
	case *float64:
		*tval = v

	case *interface{}:
		*tval = v

	default:
		return fmt.Errorf("cannot decode a double into a %T", val.Elem().Interface())
	}

	return nil
}

func decodeString(in []byte, val reflect.Value) error {
	switch tval := val.Interface().(type) {
	case *string:
		*tval = string(in)

	case *interface{}:
		*tval = string(in)

	default:
		return fmt.Errorf("cannot decode a string into a %T", val.Elem().Interface())
	}

	return nil
}

func decodeBase64(in []byte, val reflect.Value) error {
	byts, err := base64.StdEncoding.DecodeString(string(in))
	if err != nil {
		return err
	}

	switch tval := val.Interface().(type) {
	case *[]byte:
		*tval = byts

	case *interface{}:
		*tval = byts

	default:
		return fmt.Errorf("cannot decode a base64 into a %T", val.Elem().Interface())
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

			err = valueDecode(dec, field)
			if err != nil {
				return err
			}
		}

		xmlGetEndElement(dec, true)

	case reflect.Slice:
		typ := val.Elem().Type().Elem()

		for {
			err := xmlGetStartElement(dec, "value")
			if err != nil {
				// slice is over
				if err == errEndElement {
					break
				}
				return err
			}

			el := reflect.New(typ)
			err = valueDecode(dec, el)
			if err != nil {
				return err
			}

			val.Elem().Set(reflect.Append(val.Elem(), el.Elem()))
		}

	default:
		return fmt.Errorf("cannot decode an array into a %T", val.Elem().Interface())
	}

	return xmlGetEndElement(dec, true)
}

func valueDecode(dec *xml.Decoder, dest reflect.Value) error {
	if dest.Kind() != reflect.Ptr {
		return fmt.Errorf("destination is not a pointer")
	}

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

			err = decodeBool(cnt, dest)
			if err != nil {
				return err
			}

		case "int", "i4":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeInt(cnt, dest)
			if err != nil {
				return err
			}

		case "double":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeDouble(cnt, dest)
			if err != nil {
				return err
			}

		case "string":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeString(cnt, dest)
			if err != nil {
				return err
			}

		case "base64":
			cnt, err := xmlGetContent(dec)
			if err != nil {
				return err
			}

			err = decodeBase64(cnt, dest)
			if err != nil {
				return err
			}

		case "array":
			err = decodeArray(dec, dest)
			if err != nil {
				return err
			}

		default:
			return fmt.Errorf("unhandled value type: '%s'", ttok.Name.Local)
		}

		err = xmlGetEndElement(dec, true)
		if err != nil {
			return err
		}

	// string without tag
	case xml.CharData:
		err := decodeString(ttok, dest)
		if err != nil {
			return err
		}

		err = xmlGetEndElement(dec, true)
		if err != nil {
			return err
		}

	// string without tag and empty
	case xml.EndElement:
		err := decodeString([]byte{}, dest)
		if err != nil {
			return err
		}

	default:
		return fmt.Errorf("unexpected element type: %T", tok)
	}

	return nil
}

func valueEncode(w io.Writer, src reflect.Value) error {
	_, err := w.Write([]byte(`<value>`))
	if err != nil {
		return err
	}

	switch tval := src.Interface().(type) {
	case bool:
		v := "0"
		if tval {
			v = "1"
		}
		_, err := w.Write([]byte(`<boolean>` + v + `</boolean>`))
		if err != nil {
			return err
		}

	case int:
		_, err := w.Write([]byte(`<i4>` + strconv.FormatInt(int64(tval), 10) + `</i4>`))
		if err != nil {
			return err
		}

	case float64:
		_, err := w.Write([]byte(`<double>` + strconv.FormatFloat(tval, 'G', -1, 64) + `</double>`))
		if err != nil {
			return err
		}

	case string:
		_, err := w.Write([]byte(tval))
		if err != nil {
			return err
		}

	case []byte:
		_, err := w.Write([]byte(`<base64>` + base64.StdEncoding.EncodeToString(tval) + `</base64>`))
		if err != nil {
			return err
		}

	default:
		switch src.Kind() {
		case reflect.Struct:
			_, err := w.Write([]byte(`<array><data>`))
			if err != nil {
				return err
			}

			nf := src.NumField()
			for i := 0; i < nf; i++ {
				field := src.Field(i)

				err := valueEncode(w, field)
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

			le := src.Len()
			for i := 0; i < le; i++ {
				el := src.Index(i)

				err := valueEncode(w, el)
				if err != nil {
					return err
				}
			}

			_, err = w.Write([]byte(`</data></array>`))
			if err != nil {
				return err
			}

		default:
			return fmt.Errorf("unhandled value type: %s", src.Type())
		}
	}

	_, err = w.Write([]byte(`</value>`))
	if err != nil {
		return err
	}

	return nil
}
