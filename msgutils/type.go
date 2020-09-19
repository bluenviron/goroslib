package msg_utils

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msgs"
)

// Type returns the type of a message.
func Type(msg interface{}) (string, error) {
	rt := reflect.TypeOf(msg)
	if rt.Kind() == reflect.Ptr {
		rt = rt.Elem()
	}
	if rt.Kind() != reflect.Struct {
		return "", fmt.Errorf("unsupported message type '%s'", rt.String())
	}

	name := rt.Name()
	if name == "" {
		name = "Msg"
	}

	pkg := func() string {
		ft, ok := rt.FieldByName("Package")
		if !ok {
			return ""
		}

		if !ft.Anonymous {
			return ""
		}

		if ft.Type != reflect.TypeOf(msgs.Package(0)) {
			return ""
		}

		return ft.Tag.Get("ros")
	}()
	if pkg == "" {
		pkg = "goroslib"
	}

	return pkg + "/" + name, nil
}
