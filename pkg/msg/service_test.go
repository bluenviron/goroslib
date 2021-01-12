package msg

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

var casesServiceRequestResponse = []struct {
	name string
	srv  interface{}
	req  interface{}
	res  interface{}
}{
	{
		"base",
		&struct {
			ServiceReq
			ServiceRes
		}{},
		ServiceReq{},
		ServiceRes{},
	},
}

func TestServiceRequestResponse(t *testing.T) {
	for _, c := range casesServiceRequestResponse {
		t.Run(c.name, func(t *testing.T) {
			req, res, err := ServiceRequestResponse(c.srv)
			require.NoError(t, err)
			require.Equal(t, c.req, req)
			require.Equal(t, c.res, res)
		})
	}
}

var casesServiceMD5 = []struct {
	name string
	srv  interface{}
	sum  string
}{
	{
		"base",
		&struct {
			ServiceReq
			ServiceRes
		}{},
		"4fa8f09823d7ad898c6295d42385de20",
	},
}

func TestServiceMD5(t *testing.T) {
	for _, c := range casesServiceMD5 {
		t.Run(c.name, func(t *testing.T) {
			md5, err := ServiceMD5(c.srv)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}

type SrvExplicitPackage struct {
	Package `ros:"my_package"`
	ServiceReq
	ServiceRes
}

type SrvImplicitPackage struct {
	ServiceReq
	ServiceRes
}

var casesServiceType = []struct {
	name string
	srv  interface{}
	typ  string
}{
	{
		"explicit package",
		&SrvExplicitPackage{},
		"my_package/SrvExplicitPackage",
	},
	{
		"implicit package",
		&SrvImplicitPackage{},
		"goroslib/SrvImplicitPackage",
	},
}

func TestServiceType(t *testing.T) {
	for _, c := range casesServiceType {
		t.Run(c.name, func(t *testing.T) {
			typ, err := ServiceType(c.srv)
			require.NoError(t, err)
			require.Equal(t, c.typ, typ)
		})
	}
}
