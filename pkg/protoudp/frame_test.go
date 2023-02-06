package protoudp

import (
	"bytes"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestFramesForPayload(t *testing.T) {
	frames := framesForPayload(3, 2, []byte{0x01, 0x02, 0x03, 0x04})
	require.Equal(t, []*Frame{
		{
			ConnectionID: 3,
			MessageID:    2,
			BlockID:      1,
			Payload:      []byte{0x01, 0x02, 0x03, 0x04},
		},
	}, frames)

	frames = framesForPayload(2, 1, bytes.Repeat([]byte{0x01}, 1492))
	require.Equal(t, []*Frame{
		{
			ConnectionID: 2,
			MessageID:    1,
			BlockID:      1,
			Payload:      bytes.Repeat([]byte{0x01}, 1492),
		},
	}, frames)

	frames = framesForPayload(3, 2, bytes.Repeat([]byte{0x01, 0x02, 0x03, 0x04}, 500))
	require.Equal(t, []*Frame{
		{
			ConnectionID: 3,
			MessageID:    2,
			BlockID:      2,
			Payload:      bytes.Repeat([]byte{0x01, 0x02, 0x03, 0x04}, 373),
		},
		{
			Opcode:       1,
			ConnectionID: 3,
			MessageID:    2,
			BlockID:      1,
			Payload:      bytes.Repeat([]byte{0x01, 0x02, 0x03, 0x04}, 127),
		},
	}, frames)
}
