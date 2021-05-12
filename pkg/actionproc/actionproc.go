// Package actionproc contains functions to process actions.
package actionproc

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

// GoalResultFeedback returns the goal, result and feedback of an action.
func GoalResultFeedback(action interface{}) (interface{}, interface{}, interface{}, error) {
	actionv := reflect.ValueOf(action)
	if actionv.Kind() == reflect.Ptr {
		actionv = actionv.Elem()
	}
	if actionv.Kind() != reflect.Struct {
		return nil, nil, nil, fmt.Errorf("unsupported action type '%s'", actionv.Type().String())
	}

	var goal interface{}
	goalFound := false
	var res interface{}
	resFound := false

	nf := actionv.NumField()
	for i := 0; i < nf; i++ {
		ft := actionv.Field(i)

		if ft.Type() == reflect.TypeOf(msg.Package(0)) {
			continue
		}

		switch {
		case !goalFound:
			goalFound = true
			goal = ft.Interface()

		case !resFound:
			resFound = true
			res = ft.Interface()

		default:
			return goal, res, ft.Interface(), nil
		}
	}

	return nil, nil, nil, fmt.Errorf("goal, request or feedback not found")
}

// Messages returns the automatically-generated messages of an action.
func Messages(action interface{}) (interface{}, interface{}, interface{}, error) {
	goal, res, fb, err := GoalResultFeedback(action)
	if err != nil {
		return nil, nil, nil, err
	}

	goalAction := reflect.New(reflect.StructOf([]reflect.StructField{
		{
			Name: "Header",
			Type: reflect.TypeOf(std_msgs.Header{}),
		},
		{
			Name: "GoalId",
			Type: reflect.TypeOf(actionlib_msgs.GoalID{}),
		},
		{
			Name: "Goal",
			Type: reflect.TypeOf(goal),
		},
	})).Elem().Interface()

	resAction := reflect.New(reflect.StructOf([]reflect.StructField{
		{
			Name: "Header",
			Type: reflect.TypeOf(std_msgs.Header{}),
		},
		{
			Name: "Status",
			Type: reflect.TypeOf(actionlib_msgs.GoalStatus{}),
		},
		{
			Name: "Result",
			Type: reflect.TypeOf(res),
		},
	})).Elem().Interface()

	fbAction := reflect.New(reflect.StructOf([]reflect.StructField{
		{
			Name: "Header",
			Type: reflect.TypeOf(std_msgs.Header{}),
		},
		{
			Name: "Status",
			Type: reflect.TypeOf(actionlib_msgs.GoalStatus{}),
		},
		{
			Name: "Feedback",
			Type: reflect.TypeOf(fb),
		},
	})).Elem().Interface()

	return goalAction, resAction, fbAction, nil
}
