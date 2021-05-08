//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileRemoveReq struct {
	FilePath string
}

type FileRemoveRes struct {
	Success bool
	RErrno  int32
}

type FileRemove struct {
	msg.Package `ros:"mavros_msgs"`
	FileRemoveReq
	FileRemoveRes
}
