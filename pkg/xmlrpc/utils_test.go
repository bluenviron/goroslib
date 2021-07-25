package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestXMLGetProcessingInstruction(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(``)))
		err := xmlGetProcessingInstruction(dec)
		require.Equal(t, "EOF", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<tag>`)))
		err := xmlGetProcessingInstruction(dec)
		require.Equal(t, "expected xml.ProcInst, got xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<?xml version="1.0"?>`)))
		err := xmlGetProcessingInstruction(dec)
		require.NoError(t, err)
	}()
}

func TestXMLGetStartElement(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(``)))
		err := xmlGetStartElement(dec, "mytag")
		require.Equal(t, "EOF", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`</end>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.Equal(t, "XML syntax error on line 1: unexpected end element </end>", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<tag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.Equal(t, "expected xml.StartElement with name 'mytag', got 'tag'", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
	}()
}

func TestXMLGetEndElement(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(``)))
		err := xmlGetEndElement(dec, false)
		require.Equal(t, "EOF", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<tag>`)))
		err := xmlGetEndElement(dec, false)
		require.Equal(t, "expected xml.EndElement, got xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<tag>`)))
		err := xmlGetEndElement(dec, true)
		require.Equal(t, "unexpected element: xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag></othertag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlGetEndElement(dec, false)
		require.Equal(t, "XML syntax error on line 1: element <mytag> closed by </othertag>", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>test</mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlGetEndElement(dec, true)
		require.NoError(t, err)
	}()
}
