//autogenerated:yes
//nolint:revive,lll
package geographic_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type BoundingBox struct {
	msg.Package `ros:"geographic_msgs"`
	MinPt       GeoPoint
	MaxPt       GeoPoint
}