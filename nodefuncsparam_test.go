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

				err = n.SetParamBool("test_bool", true)
				require.NoError(t, err)

				err = n.SetParamInt("test_int", 123)
				require.NoError(t, err)

				err = n.SetParamString("test_string", "ABC")
				require.NoError(t, err)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			resb, err := n.GetParamBool("test_bool")
			require.NoError(t, err)
			require.Equal(t, true, resb)

			resi, err := n.GetParamInt("test_int")
			require.NoError(t, err)
			require.Equal(t, 123, resi)

			ress, err := n.GetParamString("test_string")
			require.NoError(t, err)
			require.Equal(t, "ABC", ress)
		})
	}
}
