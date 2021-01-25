package goroslib

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestSimpleActionClient(t *testing.T) {
	for _, server := range []string{
		"cpp",
		//"go",
	} {
		t.Run(server, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch server {
			case "cpp":
				p, err := newContainer("node-simpleactionserver", m.IP())
				require.NoError(t, err)
				defer p.close()
			}

			nc, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer nc.Close()

			sac, err := NewSimpleActionClient(SimpleActionClientConf{
				Node:   nc,
				Name:   "test_action",
				Action: &DoSomethingAction{},
			})
			require.NoError(t, err)
			defer sac.Close()

			sac.WaitForServer()

			activeDone := make(chan struct{})
			fbDone := make(chan struct{})
			doneDone := make(chan struct{})

			err = sac.SendGoal(SimpleActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnDone: func(res *DoSomethingActionResult) {
					close(doneDone)
				},
				OnActive: func() {
					close(activeDone)
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
					close(fbDone)
				},
			})
			require.NoError(t, err)

			<-activeDone
			<-fbDone
			<-doneDone
		})
	}
}
