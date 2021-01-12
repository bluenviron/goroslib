package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type GridCells struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Header      std_msgs.Header       //nolint:golint
	CellWidth   float32               //nolint:golint
	CellHeight  float32               //nolint:golint
	Cells       []geometry_msgs.Point //nolint:golint
}
