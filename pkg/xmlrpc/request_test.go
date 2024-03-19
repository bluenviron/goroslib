package xmlrpc

import (
	"bytes"
	"io"
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

func FuzzRequestDecode(f *testing.F) {
	f.Add([]byte(`<?xml version="1.0"?>`))
	f.Add([]byte(`<?xml version="1.0"?><methodCall>`))
	f.Add([]byte(`<?xml version="1.0"?><methodCall><methodName>`))
	f.Add([]byte(`<?xml version="1.0"?><methodCall><methodName></methodName>`))
	f.Add([]byte(
		`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName>`))
	f.Add([]byte(
		`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params>`))
	f.Add([]byte(
		`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params><param>`))
	f.Add([]byte(
		`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params><param><value>`))
	f.Add([]byte(
		`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params><param><value>aaa</value>`))

	dest := &struct {
		A string
	}{}

	f.Fuzz(func(_ *testing.T, b []byte) {
		raw, err := requestDecodeRaw(bytes.NewReader(b))
		if err == nil {
			requestDecode(raw, dest) //nolint:errcheck
		}
	})
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

func TestRequestEncodeErrors(t *testing.T) {
	for _, ca := range []struct {
		name   string
		method string
		params interface{}
		dest   io.Writer
		err    string
	}{
		{
			"open tag write error",
			"myMethod",
			nil,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"close tag write error",
			"myMethod",
			struct{}{},
			&limitedBuffer{cap: 80},
			"capacity reached",
		},
		{
			"param open tag write error",
			"myMethod",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 80},
			"capacity reached",
		},
		{
			"param content write error",
			"myMethod",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 90},
			"capacity reached",
		},
		{
			"param close tag write error",
			"myMethod",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 110},
			"capacity reached",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			err := requestEncode(ca.dest, ca.method, ca.params)
			require.EqualError(t, err, ca.err)
		})
	}
}
