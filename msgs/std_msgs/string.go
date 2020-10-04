package std_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type String struct {
	msgs.Package `ros:"std_msgs"`
	Data         string
}
