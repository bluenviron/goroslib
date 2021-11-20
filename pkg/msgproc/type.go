package msgproc

import (
	"fmt"
	"reflect"

	rmsg "github.com/aler9/goroslib/pkg/msg"
)

// Type returns the type of a message.
func Type(msg interface{}) (string, error) {
	msgt := reflect.TypeOf(msg)
	if msgt.Kind() != reflect.Struct {
		return "", fmt.Errorf("message must be a struct")
	}

	name := msgt.Name()
	if name == "" {
		name = "Msg"
	}

	pkg := func() string {
		ft, ok := msgt.FieldByName("Package")
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
