package msg

import (
	"fmt"
	"reflect"
)

// ServiceRequestResponse returns the request and response of a service.
func ServiceRequestResponse(srv interface{}) (interface{}, interface{}, error) {
	srvv := reflect.ValueOf(srv)
	if srvv.Kind() == reflect.Ptr {
		srvv = srvv.Elem()
	}
	if srvv.Kind() != reflect.Struct {
		return nil, nil, fmt.Errorf("unsupported service type '%s'", srvv.String())
	}

	var req interface{}
	reqFound := false

	nf := srvv.NumField()
	for i := 0; i < nf; i++ {
		ft := srvv.Field(i)

		if ft.Type() == reflect.TypeOf(Package(0)) {
			continue
		}

		if !reqFound {
			reqFound = true
			req = ft.Interface()

		} else {
			return req, ft.Interface(), nil
		}
	}

	return nil, nil, fmt.Errorf("service request or response not found")
}

// ServiceMD5 computes the checksum of a service.
func ServiceMD5(srv interface{}) (string, error) {
	srvt := reflect.TypeOf(srv)
	if srvt.Kind() == reflect.Ptr {
		srvt = srvt.Elem()
	}
	if srvt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported service type '%s'", srvt.String())
	}

	req, res, err := ServiceRequestResponse(srv)
	if err != nil {
		return "", err
	}

	text1, _, err := md5Text(reflect.TypeOf(req), "")
	if err != nil {
		return "", err
	}

	text2, _, err := md5Text(reflect.TypeOf(res), "")
	if err != nil {
		return "", err
	}

	return md5Sum(text1 + text2), nil
}

// ServiceType returns the type of a service.
func ServiceType(msg interface{}) (string, error) {
	return MessageType(msg)
}
