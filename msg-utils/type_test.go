package msg_utils

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
)

type MsgExplicitPackage struct {
	msgs.Package `ros:"mypackage"`
	Value        msgs.Uint16
}

type MsgImplicitPackage struct {
	Value msgs.Uint16
}

var casesType = []struct {
	name string
	msg  interface{}
	typ  string
}{
	{
		"explicit package",
		&MsgExplicitPackage{},
		"mypackage/MsgExplicitPackage",
	},
	{
		"implicit package",
		&MsgImplicitPackage{},
		"goroslib/MsgImplicitPackage",
	},
}

func TestMessageType(t *testing.T) {
	for _, c := range casesType {
		t.Run(c.name, func(t *testing.T) {
			typ, err := MessageType(c.msg)
			require.NoError(t, err)
			require.Equal(t, c.typ, typ)
		})
	}
}
