package xmlrpc

import (
	"encoding/xml"
	"io"
	"reflect"
)

func responseDecode(r io.Reader, req interface{}) error {
	dec := xml.NewDecoder(r)

	err := xmlGetProcessingInstruction(dec)
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

		err = valueDecode(dec, field)
		if err != nil {
			return err
		}
	}

	return xmlConsumeUntilEOF(dec)
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

		err = valueEncode(w, field)
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
