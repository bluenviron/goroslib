package msgproc

import (
	"fmt"
	"reflect"
)

// MD5 computes the checksum of a message.
func MD5(msg interface{}) (string, error) {
	rt := reflect.TypeOf(msg)
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", rt.String())
	}

	text, _, err := text(rt, "")
	if err != nil {
		return "", err
	}

	return md5sum(text), nil
}
