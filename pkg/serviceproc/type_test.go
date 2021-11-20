package serviceproc

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msg"
)

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
