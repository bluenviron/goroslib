package tcpros

import (
	"bytes"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestHeaderEncode(t *testing.T) {
	for _, c := range casesHeader {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := headerEncode(&buf, c.header)
			require.NoError(t, err)
			require.Equal(t, c.byts, buf.Bytes())
		})
	}
}
