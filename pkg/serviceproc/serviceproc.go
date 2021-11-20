// Package serviceproc contains functions to process services.
package serviceproc

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgproc"
)

func md5sum(text string) string {
	h := md5.New()
	h.Write([]byte(text))
	return hex.EncodeToString(h.Sum(nil))
}

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

// MD5 returns the checksum of a service.
func MD5(srv interface{}) (string, error) {
	srvt := reflect.TypeOf(srv)
	if srvt.Kind() != reflect.Struct {
		return "", fmt.Errorf("service must be a struct")
	}

	req, res, err := RequestResponse(srv)
	if err != nil {
		return "", err
	}

	text1, err := msgproc.Text(req)
	if err != nil {
		return "", err
	}

	text2, err := msgproc.Text(res)
	if err != nil {
		return "", err
	}

	return md5sum(text1 + text2), nil
}

// Type returns the type of a service.
func Type(srv interface{}) (string, error) {
	return msgproc.Type(srv)
}
