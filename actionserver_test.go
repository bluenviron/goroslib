package goroslib

/*import (
	"fmt"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestActionServer(t *testing.T) {
	for _, client := range []string{
		"cpp",
		//"go",
	} {
		t.Run(client, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			as, err := NewActionServer(ActionServerConf{
				Node:   n,
				Name:   "test_action",
				Action: &DoSomethingAction{},
			})
			require.NoError(t, err)
			defer as.Close()

			switch client {
			case "cpp":
				c, err := newContainer("node-actionclient", m.IP())
				require.NoError(t, err)
				defer c.close()
				fmt.Println(c.waitOutput())
			}
		})
	}
}*/
