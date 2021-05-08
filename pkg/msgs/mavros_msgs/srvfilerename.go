//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileRenameReq struct {
	OldPath string
	NewPath string
}

type FileRenameRes struct {
	Success bool
	RErrno  int32
}

type FileRename struct {
	msg.Package `ros:"mavros_msgs"`
	FileRenameReq
	FileRenameRes
}
