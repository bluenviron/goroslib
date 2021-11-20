package msgproc

import (
	"fmt"
	"reflect"
)

// MD5 returns the checksum of a message.
func MD5(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	text, err := textMsg(msgt)
	if err != nil {
		return "", err
	}

	return md5sum(text), nil
}
