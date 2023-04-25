package msgproc

import (
	"fmt"
	"reflect"

	rmsg "github.com/bluenviron/goroslib/v2/pkg/msg"
)

// Type returns the type of a message, which has the format "Package/Name".
func Type(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	return getType(msgt), nil
}

func getType(msgt reflect.Type) string {
	return getPackage(msgt) + "/" + getName(msgt)
}

func getPackage(msgt reflect.Type) string {
	ft, ok := msgt.FieldByName("Package")
	if ok && ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Package(0)) {
		t := ft.Tag.Get("ros")
		if t != "" {
			return t
		}
	}

	return "goroslib"
}

func getName(msgt reflect.Type) string {
	ft, ok := msgt.FieldByName("Name")
	if ok && ft.Anonymous && ft.Type == reflect.TypeOf(rmsg.Name(0)) {
		t := ft.Tag.Get("ros")
		if t != "" {
			return t
		}
	}

	name := msgt.Name()
	if name != "" {
		return name
	}

	return "Msg"
}
