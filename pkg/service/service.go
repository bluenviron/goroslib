// Package service contains utilities to process services.
package service

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/msg"
)

func md5sum(text string) string {
	h := md5.New()
	h.Write([]byte(text))
	return hex.EncodeToString(h.Sum(nil))
}

// RequestResponse returns the request and response of a service.
func RequestResponse(srv interface{}) (interface{}, interface{}, error) {
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

		if ft.Type() == reflect.TypeOf(msg.Package(0)) {
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

// MD5 computes the checksum of a service.
func MD5(srv interface{}) (string, error) {
	srvt := reflect.TypeOf(srv)
	if srvt.Kind() == reflect.Ptr {
		srvt = srvt.Elem()
	}
	if srvt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported service type '%s'", srvt.String())
	}

	req, res, err := RequestResponse(srv)
	if err != nil {
		return "", err
	}

	text1, _, err := msg.Text(reflect.TypeOf(req), "")
	if err != nil {
		return "", err
	}

	text2, _, err := msg.Text(reflect.TypeOf(res), "")
	if err != nil {
		return "", err
	}

	return md5sum(text1 + text2), nil
}

// Type returns the type of a service.
func Type(srv interface{}) (string, error) {
	return msg.Type(srv)
}
