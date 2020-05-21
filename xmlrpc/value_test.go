package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"io"
	"reflect"
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
		"bool",
		[]byte("<value><boolean>1</boolean></value>"),
		[]byte("<value><boolean>1</boolean></value>"),
		true,
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
		"string without type",
		[]byte("<value>testval</value>"),
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
		"string empty without type",
		[]byte("<value></value>"),
		[]byte("<value></value>"),
		"",
	},
	{
		"array as struct",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value>`),
		Substruct{
			"test1",
			"test2",
			123,
		},
	},
	{
		"array as slice",
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
}

func TestValueDecode(t *testing.T) {
	for _, c := range casesValue {
		t.Run(c.name, func(t *testing.T) {
			dec := xml.NewDecoder(bytes.NewReader(c.bdec))

			err := xmlGetStartElement(dec, "value")
			require.NoError(t, err)

			v := reflect.New(reflect.TypeOf(c.v))
			err = decodeValue(dec, reflect.ValueOf(v.Interface()))
			require.NoError(t, err)

			require.Equal(t, c.v, v.Elem().Interface())
			_, err = dec.Token()

			require.Equal(t, io.EOF, err)
		})
	}
}

func TestValueEncode(t *testing.T) {
	for _, c := range casesValue {
		t.Run(c.name, func(t *testing.T) {
			var b bytes.Buffer
			err := encodeValue(&b, reflect.ValueOf(c.v))
			require.NoError(t, err)
			require.Equal(t, c.benc, b.Bytes())
		})
	}
}
