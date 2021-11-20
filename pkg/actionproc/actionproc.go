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
