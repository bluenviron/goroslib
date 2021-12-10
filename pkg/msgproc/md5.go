package msgproc

import (
	"fmt"
	"reflect"
)

// MD5 returns the checksum of a message.
func MD5(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	return MD5FromReflect(msgt)
}

// MD5FromReflect returns the checksum of a message from a reflected type.
func MD5FromReflect(msgt reflect.Type) (string, error) {
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	text, err := textMsg(msgt)
	if err != nil {
		return "", err
	}

	return md5sum(text), nil
}
