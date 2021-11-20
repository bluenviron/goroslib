package actionproc

import (
	"reflect"

	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

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
