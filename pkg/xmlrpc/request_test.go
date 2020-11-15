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

func TestRequestEncode(t *testing.T) {
	for _, c := range casesRequest {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := requestEncode(&buf, c.method, c.params)
			require.NoError(t, err)
			require.Equal(t, c.benc, buf.Bytes())
		})
	}
}
