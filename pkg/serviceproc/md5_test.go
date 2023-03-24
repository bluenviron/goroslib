package serviceproc

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

func TestMD5(t *testing.T) {
	for _, ca := range []struct {
		name string
		srv  interface{}
		sum  string
	}{
		{
			"base",
			struct {
				msg.Package `ros:"my_package"`
				ServiceReq
				ServiceRes
			}{},
			"4fa8f09823d7ad898c6295d42385de20",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			md5, err := MD5(ca.srv)
			require.NoError(t, err)
			require.Equal(t, ca.sum, md5)
		})
	}
}

func TestMD5Errors(t *testing.T) {
	for _, ca := range []struct {
		name string
		srv  interface{}
		err  string
	}{
		{
			"wrong type",
			123,
			"service must be a struct",
		},
		{
			"invalid service",
			struct {
				A int
			}{},
			"request or response not found",
		},
		{
			"invalid request",
			struct {
				A int
				B int
			}{},
			"message must be a struct",
		},
		{
			"invalid response",
			struct {
				A struct{}
				B int
			}{},
			"message must be a struct",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			_, err := MD5(ca.srv)
			require.EqualError(t, err, ca.err)
		})
	}
}
