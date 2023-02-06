package serviceproc

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/msg"
)

// RequestResponse returns the request and response of a service.
func RequestResponse(srv interface{}) (interface{}, interface{}, error) {
	srvt := reflect.TypeOf(srv)
	if srvt.Kind() != reflect.Struct {
		return nil, nil, fmt.Errorf("service must be a struct")
	}

	var req interface{}
	reqFound := false

	nf := srvt.NumField()
	for i := 0; i < nf; i++ {
		ft := srvt.Field(i)

		if ft.Type == reflect.TypeOf(msg.Package(0)) {
			continue
		}

		if !reqFound {
			reqFound = true
			req = reflect.Zero(ft.Type).Interface()
		} else {
			return req, reflect.Zero(ft.Type).Interface(), nil
		}
	}

	return nil, nil, fmt.Errorf("request or response not found")
}
