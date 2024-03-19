package xmlrpc

import (
	"bytes"
	"io"
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

func FuzzResponseDecode(f *testing.F) {
	f.Add([]byte(``))
	f.Add([]byte(`<othertag>`))
	f.Add([]byte(`<?xml version="1.0"?>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params><param>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params><param><value>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params><param><value><array>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params><param><value><array><data>`))
	f.Add([]byte(`<?xml version="1.0"?><methodResponse><params><param><value><array><data><value>asd`))

	dest := &struct {
		A string
	}{}

	f.Fuzz(func(_ *testing.T, b []byte) {
		responseDecode(bytes.NewReader(b), dest) //nolint:errcheck
	})
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

func TestResponseEncodeErrors(t *testing.T) {
	for _, ca := range []struct {
		name   string
		params interface{}
		dest   io.Writer
		err    string
	}{
		{
			"open tag write error",
			nil,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"close tag write error",
			struct{}{},
			&limitedBuffer{cap: 80},
			"capacity reached",
		},
		{
			"field write error",
			struct {
				Param string
			}{"testing"},
			&limitedBuffer{cap: 80},
			"capacity reached",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			err := responseEncode(ca.dest, ca.params)
			require.EqualError(t, err, ca.err)
		})
	}
}
