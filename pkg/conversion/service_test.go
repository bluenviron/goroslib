package conversion

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

const testSrv = `sensor_msgs/CameraInfo camera_info # The camera_info to store
---
bool success          # True if the call succeeded
string status_message # Used to give details about success
`

const testSrvGo = `//autogenerated:yes
//nolint:revive,lll
package main

import (
    "github.com/bluenviron/goroslib/v2/pkg/msg"
    "github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs"
)


type MysrvReq struct {
    msg.Package ` + "`" + `ros:"main"` + "`" + `
    CameraInfo sensor_msgs.CameraInfo
}



type MysrvRes struct {
    msg.Package ` + "`" + `ros:"main"` + "`" + `
    Success bool
    StatusMessage string
}

type mysrv struct {
    msg.Package ` + "`" + `ros:"my_package"` + "`" + `
    mysrvReq
    mysrvRes
}
`

func TestService(t *testing.T) {
	dir, err := os.MkdirTemp("", "goroslib")
	require.NoError(t, err)
	defer os.RemoveAll(dir)

	fpath := filepath.Join(dir, "mysrv.srv")
	err = os.WriteFile(fpath, []byte(testSrv), 0o644)
	require.NoError(t, err)

	var buf bytes.Buffer
	err = ImportService(fpath, "main", "my_package", &buf)
	require.NoError(t, err)
	require.Equal(t, testSrvGo, buf.String())
}
