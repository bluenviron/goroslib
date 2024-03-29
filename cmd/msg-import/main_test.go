package main

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

const testMsg = `int8 BACKINGUP = 1
int8 NEEDS_UNPLUGGING = 2
int8 NEEDS_PLUGGING = 3
int8 NEEDS_UNPLUGGING_BADLY = 4
int8 NEEDS_PLUGGING_BADLY = 5

# Sound identifiers that have special meaning
int8 ALL = -1 # Only legal with PLAY_STOP
int8 PLAY_FILE = -2
int8 SAY = -3

int8 sound # Selects which sound to play (see above)

# Commands
int8 PLAY_STOP = 0 # Stop this sound from playing
int8 PLAY_ONCE = 1 # Play the sound once
int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs

int8 command # Indicates what to do with the sound

# Volume at which to play the sound, with 0 as mute and 1.0 as 100%.
float32 volume

string arg # file name or text to say
string arg2 # other arguments
`

func TestRun(t *testing.T) {
	dir, err := os.MkdirTemp("", "goroslib")
	require.NoError(t, err)
	defer os.RemoveAll(dir)

	fpath := filepath.Join(dir, "mymessage.msg")
	err = os.WriteFile(fpath, []byte(testMsg), 0o644)
	require.NoError(t, err)

	var buf bytes.Buffer
	err = run([]string{fpath}, &buf)
	require.NoError(t, err)
	require.NotEqual(t, 0, buf.Len())
}
