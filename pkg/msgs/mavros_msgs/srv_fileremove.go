//autogenerated:yes
//nolint:revive,lll
package mavros_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type FileRemoveReq struct {
	msg.Package `ros:"mavros_msgs"`
	FilePath    string
}

type FileRemoveRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	RErrno      int32
}

type FileRemove struct {
	msg.Package `ros:"mavros_msgs"`
	FileRemoveReq
	FileRemoveRes
}