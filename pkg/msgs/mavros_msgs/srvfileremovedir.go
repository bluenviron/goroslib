//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileRemoveDirReq struct {
	DirPath string
}

type FileRemoveDirRes struct {
	Success bool
	RErrno  int32
}

type FileRemoveDir struct {
	msg.Package `ros:"mavros_msgs"`
	FileRemoveDirReq
	FileRemoveDirRes
}
