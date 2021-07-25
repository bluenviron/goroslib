package xmlrpc

import (
	"bytes"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

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
			`<value><double>-1.324543</double></value>` +
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
			`<value><double>-1.324543</double></value>` +
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
				-1.324543,
			},
			"testing",
		},
	},
}

func TestResponseDecode(t *testing.T) {
	for _, ca := range casesResponse {
		t.Run(ca.name, func(t *testing.T) {
			params := reflect.New(reflect.TypeOf(ca.params))
			err := responseDecode(bytes.NewReader(ca.bdec), params.Interface())
			require.NoError(t, err)
			require.Equal(t, ca.params, params.Elem().Interface())
		})
	}
}

func TestResponseEncode(t *testing.T) {
	for _, ca := range casesResponse {
		t.Run(ca.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := responseEncode(&buf, ca.params)
			require.NoError(t, err)
			require.Equal(t, ca.benc, buf.Bytes())
		})
	}
}

func TestResponseDecodeErrors(t *testing.T) {
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
			"missing method response",
			[]byte(`<?xml version="1.0"?><othertag>`),
			"expected xml.StartElement with name 'methodResponse', got 'othertag'",
		},
		{
			"missing params",
			[]byte(`<?xml version="1.0"?><methodResponse><othertag>`),
			"expected xml.StartElement with name 'params', got 'othertag'",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			params := reflect.New(reflect.TypeOf(struct{}{}))
			err := responseDecode(bytes.NewReader(ca.enc), params.Interface())
			require.Equal(t, ca.err, err.Error())
		})
	}
}
