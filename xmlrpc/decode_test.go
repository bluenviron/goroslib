package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"io"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

type Substruct struct {
	Param1 string
	Param2 string
	Param3 int
}

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

var casesRequest = []struct {
	name   string
	bdec   []byte
	benc   []byte
	method string
	params interface{}
}{
	{
		"base",
		[]byte(`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params>` +
			`<param><value><string></string></value></param>` +
			`<param><value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value></param>` +
			`<param><value><boolean>1</boolean></value></param>` +
			`</params></methodCall>`),
		[]byte(`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params>` +
			`<param><value></value></param>` +
			`<param><value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value></param>` +
			`<param><value><boolean>1</boolean></value></param>` +
			`</params></methodCall>`),
		"testMethodName",
		struct {
			Param7 string
			Param8 Substruct
			Param9 bool
		}{
			"",
			Substruct{
				"test1",
				"test2",
				123,
			},
			true,
		},
	},
}

func TestRequestDecode(t *testing.T) {
	for _, c := range casesRequest {
		t.Run(c.name, func(t *testing.T) {
			raw, err := requestDecodeRaw(bytes.NewReader(c.bdec))
			require.NoError(t, err)
			require.Equal(t, c.method, raw.Method)

			params := reflect.New(reflect.TypeOf(c.params))
			err = raw.Decode(params.Interface())
			require.NoError(t, err)
			require.Equal(t, c.params, params.Elem().Interface())
		})
	}
}

var casesResponse = []struct {
	name   string
	bdec   []byte
	benc   []byte
	params interface{}
}{
	{
		"base",
		[]byte(`<?xml version="1.0"?><methodResponse><params>` +
			`<param><value><array><data>` +
			`<value><i4>1</i4></value>` +
			`<value></value>` +
			`<value><array><data>` +
			`<value>TCPROS</value>` +
			`<value>testing</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value>` +
			`<value>testing</value>` +
			`</data></array></value></param>` +
			`</params></methodResponse>`),
		[]byte(`<?xml version="1.0"?><methodResponse><params>` +
			`<param><value><array><data>` +
			`<value><i4>1</i4></value>` +
			`<value></value>` +
			`<value><array><data>` +
			`<value>TCPROS</value>` +
			`<value>testing</value>` +
			`<value><i4>123</i4></value>` +
			`</data></array></value>` +
			`<value>testing</value>` +
			`</data></array></value></param>` +
			`</params></methodResponse>`),
		struct {
			Param1 int
			Param2 string
			Param3 Substruct
			Param4 string
		}{
			1,
			"",
			Substruct{
				"TCPROS",
				"testing",
				123,
			},
			"testing",
		},
	},
}

func TestResponseDecode(t *testing.T) {
	for _, c := range casesResponse {
		t.Run(c.name, func(t *testing.T) {
			params := reflect.New(reflect.TypeOf(c.params))
			err := responseDecode(bytes.NewReader(c.bdec), params.Interface())
			require.NoError(t, err)
			require.Equal(t, c.params, params.Elem().Interface())
		})
	}
}
