package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

var casesValue = []struct {
	name string
	bdec []byte
	benc []byte
	v    interface{}
}{
	{
		"bool true",
		[]byte("<value><boolean>1</boolean></value>"),
		[]byte("<value><boolean>1</boolean></value>"),
		true,
	},
	{
		"bool false",
		[]byte("<value><boolean>0</boolean></value>"),
		[]byte("<value><boolean>0</boolean></value>"),
		false,
	},
	{
		"int",
		[]byte("<value><i4>-32</i4></value>"),
		[]byte("<value><i4>-32</i4></value>"),
		-32,
	},
	{
		"int with type int",
		[]byte("<value><int>-32</int></value>"),
		[]byte("<value><i4>-32</i4></value>"),
		-32,
	},
	{
		"double",
		[]byte("<value><double>-1.324543</double></value>"),
		[]byte("<value><double>-1.324543</double></value>"),
		-1.324543,
	},
	{
		"string",
		[]byte("<value><string>testval</string></value>"),
		[]byte("<value>testval</value>"),
		"testval",
	},
	{
		"string empty",
		[]byte("<value><string></string></value>"),
		[]byte("<value></value>"),
		"",
	},
	{
		"string without tag",
		[]byte("<value>testval</value>"),
		[]byte("<value>testval</value>"),
		"testval",
	},
	{
		"string without tag, empty",
		[]byte("<value></value>"),
		[]byte("<value></value>"),
		"",
	},
	{
		"base64",
		[]byte("<value><base64>AQIDBA==</base64></value>"),
		[]byte("<value><base64>AQIDBA==</base64></value>"),
		[]byte("\x01\x02\x03\x04"),
	},
	{
		"array as struct",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		struct {
			Param1 string
			Param2 string
			Param3 int
			Param4 float64
			Param5 []byte
		}{
			"test1",
			"test2",
			123,
			-1.324543,
			[]byte{0x01, 0x02, 0x03, 0x04},
		},
	},
	{
		"array as slice, single type",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`</data></array></value>`),
		[]string{"test1", "test2"},
	},
	{
		"array as slice, multi type",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]interface{}{"test1", "test2", 123, -1.324543, []byte("\x01\x02\x03\x04")},
	},
}

func TestValueDecode(t *testing.T) {
	for _, ca := range casesValue {
		t.Run(ca.name, func(t *testing.T) {
			// decode to typed variable
			func() {
				dec := xml.NewDecoder(bytes.NewReader(ca.bdec))

				err := xmlGetStartElement(dec, "value")
				require.NoError(t, err)

				v := reflect.New(reflect.TypeOf(ca.v))
				err = valueDecode(dec, reflect.ValueOf(v.Interface()))
				require.NoError(t, err)

				require.Equal(t, ca.v, v.Elem().Interface())

				_, err = dec.Token()
				require.Equal(t, io.EOF, err)
			}()

			// decode to interface
			func() {
				if strings.HasPrefix(ca.name, "array") {
					return
				}

				dec := xml.NewDecoder(bytes.NewReader(ca.bdec))

				err := xmlGetStartElement(dec, "value")
				require.NoError(t, err)

				var v interface{}
				err = valueDecode(dec, reflect.ValueOf(&v))
				require.NoError(t, err)

				require.Equal(t, ca.v, v)

				_, err = dec.Token()
				require.Equal(t, io.EOF, err)
			}()
		})
	}
}

func FuzzValueDecodeToNil(f *testing.F) {
	dest := interface{}(nil)

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func FuzzValueDecodeToInterface(f *testing.F) {
	dest := new(interface{})

	f.Add([]byte(""))
	f.Add([]byte("<!-- comment -->"))
	f.Add([]byte("<invalid>"))
	f.Add([]byte("<boolean>"))
	f.Add([]byte("<boolean></boolean>"))
	f.Add([]byte("<boolean>t</boolean>"))
	f.Add([]byte("<base64>"))
	f.Add([]byte("<base64>999</base64>"))
	f.Add([]byte("<string>"))
	f.Add([]byte("<array>"))
	f.Add([]byte("<double>"))
	f.Add([]byte("<double>aaa</double>"))
	f.Add([]byte("asd</otherval>"))
	f.Add([]byte("<int>"))
	f.Add([]byte("<int>aaa</int>"))

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func FuzzValueDecodeToString(f *testing.F) {
	v := "aaaa"
	dest := &v

	f.Add([]byte("<array><data>"))
	f.Add([]byte("<int>123</int>"))
	f.Add([]byte("<double>123</double>"))
	f.Add([]byte("<base64>aGVsbG8=</base64>"))

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func FuzzValueDecodeToInteger(f *testing.F) {
	v := 123
	dest := &v

	f.Add([]byte("<boolean>1</boolean>"))
	f.Add([]byte("<string>asd</string>"))
	f.Add([]byte("asd</value>"))
	f.Add([]byte("</value>"))

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func FuzzValueDecodeToSlice(f *testing.F) {
	dest := &[]string{}

	f.Add([]byte("<array><data>"))
	f.Add([]byte("<array><data><value><int>123</int>"))

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func FuzzValueDecodeToStruct(f *testing.F) {
	dest := &struct {
		A string
	}{}

	f.Add([]byte("<array><data></data>"))
	f.Add([]byte("<array><data>"))
	f.Add([]byte("<array><data><value><int>123</int>"))

	f.Fuzz(func(t *testing.T, b []byte) {
		dec := xml.NewDecoder(bytes.NewReader(append([]byte("<value>"), b...)))
		err := xmlGetStartElement(dec, "value")
		require.NoError(t, err)
		valueDecode(dec, reflect.ValueOf(dest))
	})
}

func TestValueEncode(t *testing.T) {
	for _, ca := range casesValue {
		t.Run(ca.name, func(t *testing.T) {
			var b bytes.Buffer
			err := valueEncode(&b, reflect.ValueOf(ca.v))
			require.NoError(t, err)
			require.Equal(t, ca.benc, b.Bytes())
		})
	}
}

type limitedBuffer struct {
	cap int
	n   int
}

func (b *limitedBuffer) Write(p []byte) (int, error) {
	b.n += len(p)
	if b.n > b.cap {
		return 0, fmt.Errorf("capacity reached")
	}
	return len(p), nil
}

func TestValueEncodeError(t *testing.T) {
	for _, ca := range []struct {
		name string
		v    interface{}
		dest io.Writer
		err  string
	}{
		{
			"open tag write error",
			nil,
			&limitedBuffer{cap: 0},
			"capacity reached",
		},
		{
			"close tag write error",
			true,
			&limitedBuffer{cap: 32},
			"capacity reached",
		},
		{
			"bool write error",
			true,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"int write error",
			123,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"double write error",
			float64(123),
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"string write error",
			"testing",
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"base64 write error",
			[]byte("testing"),
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"struct open tag write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"struct field write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 20},
			"capacity reached",
		},
		{
			"struct close tag write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 50},
			"capacity reached",
		},
		{
			"slice open tag write error",
			[]string{"testing"},
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"slice element write error",
			[]string{"testing"},
			&limitedBuffer{cap: 20},
			"capacity reached",
		},
		{
			"slice close tag write error",
			[]string{"testing"},
			&limitedBuffer{cap: 50},
			"capacity reached",
		},
		{
			"unhandled value type",
			int64(123),
			bytes.NewBuffer(nil),
			"unhandled value type: int64",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			err := valueEncode(ca.dest, reflect.ValueOf(ca.v))
			require.EqualError(t, err, ca.err)
		})
	}
}
