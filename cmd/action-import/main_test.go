package main

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

const testAction = `SoundRequest sound_request
---
bool playing
time stamp
---
bool playing
time stamp
`

func TestRun(t *testing.T) {
	dir, err := os.MkdirTemp("", "goroslib")
	require.NoError(t, err)
	defer os.RemoveAll(dir)

	fpath := filepath.Join(dir, "myaction.action")
	err = os.WriteFile(fpath, []byte(testAction), 0o644)
	require.NoError(t, err)

	var buf bytes.Buffer
	err = run([]string{fpath}, &buf)
	require.NoError(t, err)
	require.NotEqual(t, 0, buf.Len())
}
