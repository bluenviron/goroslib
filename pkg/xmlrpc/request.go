package xmlrpc

import (
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
)

// RequestRaw is a raw request.
type RequestRaw struct {
	Method string
	dec    *xml.Decoder
}

// Decode transforms a RequestRaw into a Request.
func (rr *RequestRaw) Decode(req interface{}) error {
	return requestDecode(rr, req)
}

func requestDecodeRaw(r io.Reader) (*RequestRaw, error) {
	raw := &RequestRaw{
		dec: xml.NewDecoder(r),
	}

	err := xmlGetProcessingInstruction(raw.dec)
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
		return nil, fmt.Errorf("expected xml.CharData, got %T", tok)
	}
	raw.Method = string(cnt)

	err = xmlGetEndElement(raw.dec, false)
	if err != nil {
		return nil, err
	}

	return raw, nil
}

func requestDecode(raw *RequestRaw, dest interface{}) error {
	err := xmlGetStartElement(raw.dec, "params")
	if err != nil {
		return err
	}

	// read each param
	rv := reflect.ValueOf(dest).Elem()
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

		err = valueDecode(raw.dec, field)
		if err != nil {
			return err
		}

		err = xmlGetEndElement(raw.dec, true)
		if err != nil {
			return err
		}
	}

	return xmlConsumeUntilEOF(raw.dec)
}

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

		err = valueEncode(w, field)
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
