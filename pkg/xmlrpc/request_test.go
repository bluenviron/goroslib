package xmlrpc

import (
	"bytes"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

type Substruct struct {
	Param1 string
	Param2 string
	Param3 int
	Param4 float64
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
			`<value><double>-1.324543</double></value>` +
			`</data></array></value></param>` +
			`<param><value><boolean>1</boolean></value></param>` +
			`</params></methodCall>`),
		[]byte(`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params>` +
			`<param><value></value></param>` +
			`<param><value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
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
				-1.324543,
			},
			true,
		},
	},
}

func TestRequestDecode(t *testing.T) {
	for _, ca := range casesRequest {
		t.Run(ca.name, func(t *testing.T) {
			raw, err := requestDecodeRaw(bytes.NewReader(ca.bdec))
			require.NoError(t, err)
			require.Equal(t, ca.method, raw.Method)

			params := reflect.New(reflect.TypeOf(ca.params))
			err = raw.Decode(params.Interface())
			require.NoError(t, err)
			require.Equal(t, ca.params, params.Elem().Interface())
		})
	}
}

func TestRequestEncode(t *testing.T) {
	for _, ca := range casesRequest {
		t.Run(ca.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := requestEncode(&buf, ca.method, ca.params)
			require.NoError(t, err)
			require.Equal(t, ca.benc, buf.Bytes())
		})
	}
}

func TestRequestDecodeErrors(t *testing.T) {
	for _, ca := range []struct {
		name string
		enc  []byte
		err  string
	}{
		{
			"empty",
			[]byte(""),
			"EOF",
		},
		{
			"missing processing instruction",
			[]byte(`<othertag>`),
			"expected xml.ProcInst, got xml.StartElement",
		},
		{
			"missing method call",
			[]byte(`<?xml version="1.0"?><othertag>`),
			"expected xml.StartElement with name 'methodCall', got 'othertag'",
		},
		{
			"missing method name",
			[]byte(`<?xml version="1.0"?><methodCall><othertag>`),
			"expected xml.StartElement with name 'methodName', got 'othertag'",
		},
		{
			"missing method name content",
			[]byte(`<?xml version="1.0"?><methodCall><methodName></methodName>`),
			"expected xml.CharData, got xml.EndElement",
		},
		{
			"missing method name closing tag",
			[]byte(`<?xml version="1.0"?><methodCall><methodName>asd`),
			"XML syntax error on line 1: unexpected EOF",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			_, err := requestDecodeRaw(bytes.NewReader(ca.enc))
			require.Equal(t, ca.err, err.Error())
		})
	}
}
