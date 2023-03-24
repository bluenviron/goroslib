package serviceproc

import (
	"github.com/bluenviron/goroslib/v2/pkg/msgproc"
)

// Type returns the type of a service.
func Type(srv interface{}) (string, error) {
	return msgproc.Type(srv)
}
