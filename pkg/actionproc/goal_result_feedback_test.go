package actionproc

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/msg"
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
	for _, ca := range []struct {
		name string
		act  interface{}
		goal interface{}
		res  interface{}
		fb   interface{}
	}{
		{
			"base",
			struct {
				ActionGoal
				ActionResult
				ActionFeedback
			}{},
			ActionGoal{},
			ActionResult{},
			ActionFeedback{},
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			goal, res, fb, err := GoalResultFeedback(ca.act)
			require.NoError(t, err)
			require.Equal(t, ca.goal, goal)
			require.Equal(t, ca.res, res)
			require.Equal(t, ca.fb, fb)
		})
	}
}

func TestGoalResultFeedbackErrors(t *testing.T) {
	t.Run("invalid action 1", func(t *testing.T) {
		_, _, _, err := GoalResultFeedback(123)
		require.EqualError(t, err, "action must be a struct")
	})

	t.Run("invalid action 2", func(t *testing.T) {
		_, _, _, err := GoalResultFeedback(struct {
			ActionGoal
		}{})
		require.EqualError(t, err, "goal, request or feedback not found")
	})
}
