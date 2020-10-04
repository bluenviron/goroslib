package msg

import (
	"testing"

	"github.com/stretchr/testify/require"
)

type MsgExplicitPackage struct {
	Package `ros:"my_package"`
	Value   uint16
}

type MsgImplicitPackage struct {
	Value uint16
}

var casesType = []struct {
	name string
	msg  interface{}
	typ  string
}{
	{
		"explicit package",
		&MsgExplicitPackage{},
		"my_package/MsgExplicitPackage",
	},
	{
		"implicit package",
		&MsgImplicitPackage{},
		"goroslib/MsgImplicitPackage",
	},
}

func TestType(t *testing.T) {
	for _, c := range casesType {
		t.Run(c.name, func(t *testing.T) {
			typ, err := Type(c.msg)
			require.NoError(t, err)
			require.Equal(t, c.typ, typ)
		})
	}
}
