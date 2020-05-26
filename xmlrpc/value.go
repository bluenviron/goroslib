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

	switch tval := val.Interface().(type) {
	case *bool:
		*tval = v

	default:
		return fmt.Errorf("cannot decode a bool into a %T", val.Interface())
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
		return fmt.Errorf("cannot decode a int into a %T", val.Interface())
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
		return fmt.Errorf("cannot decode a double into a %T", val.Interface())
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
		return fmt.Errorf("cannot decode a string into a %T", val.Interface())
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

		err = xmlGetEndElement(dec, "data")
		if err != nil {
			return err
		}

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
		return fmt.Errorf("cannot decode an array into a %s", val.Elem().Kind())
	}

	return xmlGetEndElement(dec, "array")
}

func valueDecode(dec *xml.Decoder, val reflect.Value) error {
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

func valueEncode(w io.Writer, val reflect.Value) error {
	_, err := w.Write([]byte(`<value>`))
	if err != nil {
		return err
	}

	switch tval := val.Interface().(type) {
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

	default:
		switch val.Kind() {
		case reflect.Struct:
			_, err := w.Write([]byte(`<array><data>`))
			if err != nil {
				return err
			}

			nf := val.NumField()
			for i := 0; i < nf; i++ {
				field := val.Field(i)

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

			le := val.Len()
			for i := 0; i < le; i++ {
				el := val.Index(i)

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
			return fmt.Errorf("unhandled value type: %s", val.Kind())
		}
	}

	_, err = w.Write([]byte(`</value>`))
	if err != nil {
		return err
	}

	return nil
}
