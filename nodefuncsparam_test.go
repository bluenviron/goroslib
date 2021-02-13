package goroslib

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestNodeSetGetParam(t *testing.T) {
	for _, lang := range []string{
		"cpp",
		"go",
	} {
		t.Run(lang, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch lang {
			case "cpp":
				p, err := newContainer("node-setparam", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_set",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer n.Close()

				err = n.ParamSetBool("test_bool", true)
				require.NoError(t, err)

				err = n.ParamSetInt("test_int", 123)
				require.NoError(t, err)

				err = n.ParamSetString("test_string", "ABC")
				require.NoError(t, err)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			resb, err := n.ParamGetBool("test_bool")
			require.NoError(t, err)
			require.Equal(t, true, resb)

			resi, err := n.ParamGetInt("test_int")
			require.NoError(t, err)
			require.Equal(t, 123, resi)

			ress, err := n.ParamGetString("test_string")
			require.NoError(t, err)
			require.Equal(t, "ABC", ress)
		})
	}
}
