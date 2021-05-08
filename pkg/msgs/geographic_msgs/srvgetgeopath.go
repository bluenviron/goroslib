//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/uuid_msgs"
)

type GetGeoPathReq struct {
	Start GeoPoint
	Goal  GeoPoint
}

type GetGeoPathRes struct {
	Success  bool
	Status   string
	Plan     GeoPath
	Network  uuid_msgs.UniqueID
	StartSeg uuid_msgs.UniqueID
	GoalSeg  uuid_msgs.UniqueID
	Distance float64
}

type GetGeoPath struct {
	msg.Package `ros:"geographic_msgs"`
	GetGeoPathReq
	GetGeoPathRes
}
