package msg

import (
	"testing"

	"github.com/stretchr/testify/require"
)

var casesServiceMD5 = []struct {
	name string
	srv  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			A struct {
				A float64
				B string
			}
			B struct {
				C float64
			}
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
	A       struct {
		A float64
		B string
	}
	B struct {
		C float64
	}
}

type SrvImplicitPackage struct {
	A struct {
		A float64
		B string
	}
	B struct {
		C float64
	}
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
