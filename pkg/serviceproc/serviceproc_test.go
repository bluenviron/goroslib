package serviceproc

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msg"
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

type SrvExplicitPackage struct {
	msg.Package `ros:"my_package"`
	ServiceReq
	ServiceRes
}

type SrvImplicitPackage struct {
	ServiceReq
	ServiceRes
}

func TestType(t *testing.T) {
	for _, ca := range []struct {
		name string
		srv  interface{}
		typ  string
	}{
		{
			"explicit package",
			SrvExplicitPackage{},
			"my_package/SrvExplicitPackage",
		},
		{
			"implicit package",
			SrvImplicitPackage{},
			"goroslib/SrvImplicitPackage",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			typ, err := Type(ca.srv)
			require.NoError(t, err)
			require.Equal(t, ca.typ, typ)
		})
	}
}
