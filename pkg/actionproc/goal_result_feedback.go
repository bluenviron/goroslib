package actionproc

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/msg"
)

// GoalResultFeedback returns the goal, result and feedback of an action.
func GoalResultFeedback(action interface{}) (interface{}, interface{}, interface{}, error) {
	actiont := reflect.TypeOf(action)
	if actiont.Kind() != reflect.Struct {
		return nil, nil, nil, fmt.Errorf("action must be a struct")
	}

	var goal interface{}
	goalFound := false
	var res interface{}
	resFound := false

	nf := actiont.NumField()
	for i := 0; i < nf; i++ {
		ft := actiont.Field(i)

		if ft.Type == reflect.TypeOf(msg.Package(0)) {
			continue
		}

		switch {
		case !goalFound:
			goalFound = true
			goal = reflect.Zero(ft.Type).Interface()

		case !resFound:
			resFound = true
			res = reflect.Zero(ft.Type).Interface()

		default:
			return goal, res, reflect.Zero(ft.Type).Interface(), nil
		}
	}

	return nil, nil, nil, fmt.Errorf("goal, request or feedback not found")
}
