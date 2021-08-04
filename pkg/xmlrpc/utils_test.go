package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestXMLGetProcessingInstruction(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<?xml version="1.0"?>`)))
		err := xmlGetProcessingInstruction(dec)
		require.NoError(t, err)
	}()
}

func TestXMLGetProcessingInstructionErrors(t *testing.T) {
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
}

func TestXMLGetStartElement(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`string<mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
	}()
}

func TestXMLGetStartElementErrors(t *testing.T) {
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
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<!-- comment -->`)))
		err := xmlGetStartElement(dec, "mytag")
		require.Equal(t, "unexpected element type: xml.Comment", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<tag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.Equal(t, "expected xml.StartElement with name 'mytag', got 'tag'", err.Error())
	}()
}

func TestXMLGetEndElement(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>test</mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlGetEndElement(dec, true)
		require.NoError(t, err)
	}()
}

func TestXMLGetEndElementErrors(t *testing.T) {
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
		require.Equal(t, "unexpected element type: xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag></othertag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlGetEndElement(dec, false)
		require.Equal(t, "XML syntax error on line 1: element <mytag> closed by </othertag>", err.Error())
	}()
}

func TestXMLGetContent(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>content</mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		cnt, err := xmlGetContent(dec)
		require.NoError(t, err)
		require.Equal(t, "content", string(cnt))
	}()
}

func TestXMLGetContentErrors(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		_, err = xmlGetContent(dec)
		require.Equal(t, "XML syntax error on line 1: unexpected EOF", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag><mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		_, err = xmlGetContent(dec)
		require.Equal(t, "unexpected element type: xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>content`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		_, err = xmlGetContent(dec)
		require.Equal(t, "XML syntax error on line 1: unexpected EOF", err.Error())
	}()
}

func TestXMLConsumeUntilEOF(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>ok</mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlConsumeUntilEOF(dec)
		require.NoError(t, err)
	}()
}

func TestXMLConsumeUntilEOFErrors(t *testing.T) {
	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>`)))
		err := xmlConsumeUntilEOF(dec)
		require.Equal(t, "unexpected element type: xml.StartElement", err.Error())
	}()

	func() {
		dec := xml.NewDecoder(bytes.NewReader([]byte(`<mytag>`)))
		err := xmlGetStartElement(dec, "mytag")
		require.NoError(t, err)
		err = xmlConsumeUntilEOF(dec)
		require.Equal(t, "XML syntax error on line 1: unexpected EOF", err.Error())
	}()
}
