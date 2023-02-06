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
			m := newContainerMaster(t)
			defer m.close()

			switch lang {
			case "cpp":
				p := newContainer(t, "node-setparam", m.IP())
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

				err = n.ParamSetFloat64("test_double", 32.5)
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

			res1, err := n.ParamGetInt("test_int")
			require.NoError(t, err)
			require.Equal(t, 123, res1)

			res2, err := n.ParamGetString("test_string")
			require.NoError(t, err)
			require.Equal(t, "ABC", res2)

			res3, err := n.ParamGetFloat64("test_double")
			require.NoError(t, err)
			require.Equal(t, 32.5, res3)
		})
	}
}
