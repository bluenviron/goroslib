package xmlrpc

import (
	"encoding/xml"
	"fmt"
	"io"
)

var errEndElement = fmt.Errorf("unexpected EndElement")

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
