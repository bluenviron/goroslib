//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/uuid_msgs"
)

type GetRoutePlanReq struct {
	Network uuid_msgs.UniqueID
	Start   uuid_msgs.UniqueID
	Goal    uuid_msgs.UniqueID
}

type GetRoutePlanRes struct {
	Success bool
	Status  string
	Plan    RoutePath
}

type GetRoutePlan struct {
	msg.Package `ros:"geographic_msgs"`
	GetRoutePlanReq
	GetRoutePlanRes
}
