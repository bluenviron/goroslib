package xmlrpc

import (
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
	"strconv"
)

var errEndElement = fmt.Errorf("unexpected EndElement")

type RequestRaw struct {
	Method string
	dec    *xml.Decoder
}

func (rr *RequestRaw) Decode(req interface{}) error {
	return requestDecode(rr, req)
}

func requestDecodeRaw(r io.Reader) (*RequestRaw, error) {
	raw := &RequestRaw{
		dec: xml.NewDecoder(r),
	}

	err := xmlGetProcInst(raw.dec)
	if err != nil {
		return nil, err
	}

	err = xmlGetStartElement(raw.dec, "methodCall")
	if err != nil {
		return nil, err
	}

	err = xmlGetStartElement(raw.dec, "methodName")
	if err != nil {
		return nil, err
	}

	tok, err := raw.dec.Token()
	if err != nil {
		return nil, err
	}

	cnt, ok := tok.(xml.CharData)
	if !ok {
		return nil, fmt.Errorf("expected CharData, got %T", tok)
	}
	raw.Method = string(cnt)

	tok, err = raw.dec.Token()
	if err != nil {
		return nil, err
	}

	_, ok = tok.(xml.EndElement)
	if !ok {
		return nil, fmt.Errorf("expected EndElement, got %T", tok)
	}

	return raw, nil
}

func requestDecode(raw *RequestRaw, req interface{}) error {
	err := xmlGetStartElement(raw.dec, "params")
	if err != nil {
		return err
	}

	// read each param
	rv := reflect.ValueOf(req).Elem()
	nf := rv.NumField()
	for i := 0; i < nf; i++ {
		field := rv.Field(i).Addr()

		err = xmlGetStartElement(raw.dec, "param")
		if err != nil {
			return err
		}

		err := xmlGetStartElement(raw.dec, "value")
		if err != nil {
			return err
		}

		err = decodeValue(raw.dec, field)
		if err != nil {
			return err
		}

		err = xmlGetEndElement(raw.dec, "param")
		if err != nil {
			return err
		}
	}

	return xmlConsumeUntilEOF(raw.dec)
}

func responseDecode(r io.Reader, req interface{}) error {
	dec := xml.NewDecoder(r)

	err := xmlGetProcInst(dec)
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "methodResponse")
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "params")
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "param")
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "value")
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "array")
	if err != nil {
		return err
	}

	err = xmlGetStartElement(dec, "data")
	if err != nil {
		return err
	}

	// read each value
	rv := reflect.ValueOf(req).Elem()
	nf := rv.NumField()
	for i := 0; i < nf; i++ {
		field := rv.Field(i).Addr()

		err := xmlGetStartElement(dec, "value")
		if err != nil {
			return err
		}

		err = decodeValue(dec, field)
		if err != nil {
			return err
		}
	}

	return xmlConsumeUntilEOF(dec)
}

func xmlGetProcInst(dec *xml.Decoder) error {
	tok, err := dec.Token()
	if err != nil {
		return err
	}

	_, ok := tok.(xml.ProcInst)
	if !ok {
		return fmt.Errorf("expected ProcInst, got %T", tok)
	}

	return nil
}

func xmlGetStartElement(dec *xml.Decoder, name string) error {
	for {
		tok, err := dec.Token()
		if err != nil {
			return err
		}

		switch ttok := tok.(type) {
		// skip spaces, newlines, etc
		case xml.CharData:

		case xml.StartElement:
			if ttok.Name.Local != name {
				return fmt.Errorf("expected StartElement with name '%s', got '%s'", name, ttok.Name.Local)
			}
			return nil

		case xml.EndElement:
			return errEndElement

		default:
			return fmt.Errorf("unexpected element type: %T", tok)
		}
	}
}

func xmlGetEndElement(dec *xml.Decoder, name string) error {
	for {
		tok, err := dec.Token()
		if err != nil {
			return err
		}

		switch ttok := tok.(type) {
		// skip spaces, newlines, etc
		case xml.CharData:

		case xml.EndElement:
			if ttok.Name.Local != name {
				return fmt.Errorf("expected EndElement with name '%s', got '%s'", name, ttok.Name.Local)
			}
			return nil

		default:
			return fmt.Errorf("unexpected element type: %T", tok)
		}
	}
}

func xmlGetContent(dec *xml.Decoder) ([]byte, error) {
	tok, err := dec.Token()
	if err != nil {
		return nil, err
	}

	switch ttok := tok.(type) {
	// non-empty content
	case xml.CharData:
		ret := append([]byte(nil), ttok...)

		// exit from current tag
		err = dec.Skip()
		if err != nil {
			return nil, err
		}

		return ret, nil

	// empty content
	case xml.EndElement:
		return nil, nil

	default:
		return nil, fmt.Errorf("unexpected element type: %T", tok)
	}
}

func xmlConsumeUntilEOF(dec *xml.Decoder) error {
	for {
		tok, err := dec.Token()
		if err != nil {
			if err == io.EOF {
				return nil
			}
			return err
		}

		switch tok.(type) {
		// skip spaces, newlines, etc
		case xml.CharData:

		case xml.EndElement:

		default:
			return fmt.Errorf("unexpected element type: %T", tok)
		}
	}
}

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
