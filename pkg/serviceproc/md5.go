package serviceproc

import (
	"crypto/md5"
	"encoding/hex"
	"fmt"
	"reflect"

	"github.com/bluenviron/goroslib/v2/pkg/msgproc"
)

func md5sum(text string) string {
	h := md5.New()
	h.Write([]byte(text))
	return hex.EncodeToString(h.Sum(nil))
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
