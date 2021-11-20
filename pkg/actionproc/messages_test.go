package actionproc

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgproc"
)

func TestMessages(t *testing.T) {
	goalAction, resAction, fbAction, err := Messages(DoSomethingAction{})
	require.NoError(t, err)

	cur, err := msgproc.MD5(goalAction)
	require.NoError(t, err)
	require.Equal(t, "3af8ef92ce0ecab8a808095234c6d844", cur)

	cur, err = msgproc.MD5(resAction)
	require.NoError(t, err)
	require.Equal(t, "f33093186ce0321119e729ea6e5846fc", cur)

	cur, err = msgproc.MD5(fbAction)
	require.NoError(t, err)
	require.Equal(t, "25bfb21ced59f4f9490772d56f6961f4", cur)
}

func TestMessagesError(t *testing.T) {
	t.Run("invalid action", func(t *testing.T) {
		_, _, _, err := Messages(123)
		require.EqualError(t, err, "action must be a struct", err)
	})
}
