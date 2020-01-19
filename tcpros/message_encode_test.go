package tcpros

import (
	"bytes"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestMessageEncode(t *testing.T) {
	for _, c := range casesMessage {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := messageEncode(&buf, c.msg)
			require.NoError(t, err)
			require.Equal(t, c.byts, buf.Bytes())
		})
	}
}
