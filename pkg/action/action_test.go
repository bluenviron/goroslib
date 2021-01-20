package action

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msg"
)

type ActionGoal struct {
	A float64
	B string
}

type ActionResult struct {
	C float64
}

type ActionFeedback struct {
	D int32
}

type DoSomethingActionGoal struct {
	Input uint32
}

type DoSomethingActionResult struct {
	Output uint32
}

type DoSomethingActionFeedback struct {
	PercentComplete float32
}

type DoSomethingAction struct {
	msg.Package `ros:"shared_actions"`
	DoSomethingActionGoal
	DoSomethingActionResult
	DoSomethingActionFeedback
}

func TestGoalResultFeedback(t *testing.T) {
	for _, c := range []struct {
		name string
		act  interface{}
		goal interface{}
		res  interface{}
		fb   interface{}
	}{
		{
			"base",
			&struct {
				ActionGoal
				ActionResult
				ActionFeedback
			}{},
			ActionGoal{},
			ActionResult{},
			ActionFeedback{},
		},
	} {
		t.Run(c.name, func(t *testing.T) {
			goal, res, fb, err := GoalResultFeedback(c.act)
			require.NoError(t, err)
			require.Equal(t, c.goal, goal)
			require.Equal(t, c.res, res)
			require.Equal(t, c.fb, fb)
		})
	}
}

func TestMessages(t *testing.T) {
	goalAction, resAction, fbAction, err := Messages(&DoSomethingAction{})
	require.NoError(t, err)

	cur, err := msg.MD5(goalAction)
	require.NoError(t, err)
	require.Equal(t, "3af8ef92ce0ecab8a808095234c6d844", cur)

	cur, err = msg.MD5(resAction)
	require.NoError(t, err)
	require.Equal(t, "f33093186ce0321119e729ea6e5846fc", cur)

	cur, err = msg.MD5(fbAction)
	require.NoError(t, err)
	require.Equal(t, "25bfb21ced59f4f9490772d56f6961f4", cur)
}
