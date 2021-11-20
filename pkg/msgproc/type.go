package msgproc

import (
	"fmt"
	"reflect"

	rmsg "github.com/aler9/goroslib/pkg/msg"
)

// Type returns the type of a message.
func Type(msg interface{}) (string, error) {
	rt := reflect.TypeOf(msg)
	if rt.Kind() != reflect.Ptr {
		return "", fmt.Errorf("message must be a pointer")
	}
	rt = rt.Elem()
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a pointer to a struct")
	}

	name := rt.Name()
	if name == "" {
		name = "Msg"
	}

	pkg := func() string {
		ft, ok := rt.FieldByName("Package")
		if !ok || !ft.Anonymous || ft.Type != reflect.TypeOf(rmsg.Package(0)) {
			return ""
		}

		return ft.Tag.Get("ros")
	}()
	if pkg == "" {
		pkg = "goroslib"
	}

	return pkg + "/" + name, nil
}
