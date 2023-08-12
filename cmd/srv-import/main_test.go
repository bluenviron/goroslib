package main

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

func TestRun(t *testing.T) {
	dir, err := os.MkdirTemp("", "goroslib")
	require.NoError(t, err)
	defer os.RemoveAll(dir)

	fpath := filepath.Join(dir, "mysrv.srv")
	err = os.WriteFile(fpath, []byte(testSrv), 0o644)
	require.NoError(t, err)

	var buf bytes.Buffer
	err = run([]string{fpath}, &buf)
	require.NoError(t, err)
	require.NotEqual(t, 0, buf.Len())
}
