package serviceproc

import (
	"github.com/aler9/goroslib/pkg/msgproc"
)

// Type returns the type of a service.
func Type(srv interface{}) (string, error) {
	return msgproc.Type(srv)
}
