package xmlrpc

import (
	"bytes"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

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

func TestResponseEncode(t *testing.T) {
	for _, c := range casesResponse {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := responseEncode(&buf, c.params)
			require.NoError(t, err)
			require.Equal(t, c.benc, buf.Bytes())
		})
	}
}
