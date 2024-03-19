package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/msgs/rosgraph_msgs"
)

func TestNodeLog(t *testing.T) {
	t.Run("rosout", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n1, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib1",
			MasterAddress: m.IP() + ":11311",
			LogLevel:      LogLevelDebug,
		})
		require.NoError(t, err)
		defer n1.Close()

		n2, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib2",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n2.Close()

		recv := 0
		done := make(chan struct{})

		sub, err := NewSubscriber(SubscriberConf{
			Node:  n2,
			Topic: "/rosout",
			Callback: func(msg *rosgraph_msgs.Log) {
				switch msg.Level {
				case rosgraph_msgs.Log_INFO,
					rosgraph_msgs.Log_WARN,
					rosgraph_msgs.Log_ERROR,
					rosgraph_msgs.Log_FATAL:
					recv++

					if recv >= 4 {
						close(done)
					}
				}
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		time.Sleep(500 * time.Millisecond)

		n1.Log(LogLevelInfo, "test info")
		n1.Log(LogLevelWarn, "test warn")
		n1.Log(LogLevelError, "test error")
		n1.Log(LogLevelFatal, "test fatal")

		<-done
	})

	t.Run("callback", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		recv := 0
		done := make(chan struct{})

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
			LogLevel:      LogLevelDebug,
			OnLog: func(level LogLevel, _ string) {
				switch level {
				case LogLevelInfo,
					LogLevelWarn,
					LogLevelError,
					LogLevelFatal:
					recv++

					if recv >= 4 {
						close(done)
					}
				}
			},
		})
		require.NoError(t, err)
		defer n.Close()

		n.Log(LogLevelInfo, "test info")
		n.Log(LogLevelWarn, "test warn")
		n.Log(LogLevelError, "test error")
		n.Log(LogLevelFatal, "test fatal")

		<-done
	})
}
