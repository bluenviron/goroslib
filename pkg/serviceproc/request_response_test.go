package serviceproc

import (
	"testing"

	"github.com/stretchr/testify/require"
)

type ServiceReq struct {
	A float64
	B string
}

type ServiceRes struct {
	C float64
}

func TestRequestResponse(t *testing.T) {
	for _, ca := range []struct {
		name string
		srv  interface{}
		req  interface{}
		res  interface{}
	}{
		{
			"base",
			struct {
				ServiceReq
				ServiceRes
			}{},
			ServiceReq{},
			ServiceRes{},
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			req, res, err := RequestResponse(ca.srv)
			require.NoError(t, err)
			require.Equal(t, ca.req, req)
			require.Equal(t, ca.res, res)
		})
	}
}

func TestRequestResponseErrors(t *testing.T) {
	for _, ca := range []struct {
		name string
		srv  interface{}
		err  string
	}{
		{
			"invalid type",
			123,
			"service must be a struct",
		},
		{
			"missing response",
			struct {
				A struct{}
			}{},
			"request or response not found",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			_, _, err := RequestResponse(ca.srv)
			require.EqualError(t, err, ca.err)
		})
	}
}
