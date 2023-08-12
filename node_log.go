package goroslib

import (
	"fmt"
	"os"
	"time"

	"github.com/gookit/color"

	"github.com/bluenviron/goroslib/v2/pkg/msgs/rosgraph_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

// LogLevel is the level of a log message.
type LogLevel int

// standard log levels (http://wiki.ros.org/roscpp/Overview/Logging)
const (
	LogLevelDebug LogLevel = iota + 1
	LogLevelInfo
	LogLevelWarn
	LogLevelError
	LogLevelFatal
)

// LogDestination is the destination of a log message.
type LogDestination int

// available destinations.
const (
	LogDestinationConsole  LogDestination = 0x01
	LogDestinationRosout   LogDestination = 0x02
	LogDestinationCallback LogDestination = 0x04
)

// Log writes a log message.
// This is implemented like the reference C++ implementation,
// except for the log file.
// (http://wiki.ros.org/roscpp/Overview/Logging)
func (n *Node) Log(level LogLevel, format string, args ...interface{}) {
	if level < n.conf.LogLevel {
		return
	}

	msg := fmt.Sprintf(format, args...)
	now := time.Now()

	if (n.conf.LogDestinations & LogDestinationConsole) != 0 {
		formatted := msg

		switch level {
		case LogLevelDebug:
			formatted = "[DEBUG] " + formatted
		case LogLevelInfo:
			formatted = "[INFO] " + formatted
		case LogLevelWarn:
			formatted = "[WARN] " + formatted
		case LogLevelError:
			formatted = "[ERROR] " + formatted
		case LogLevelFatal:
			formatted = "[FATAL] " + formatted
		}

		formatted = now.Format("[2006/01/02 15:04:05]") + " " + formatted

		switch level {
		case LogLevelDebug:
			os.Stdout.WriteString(color.RenderString(color.Gray.Code(), formatted) + "\n")

		case LogLevelInfo:
			os.Stdout.WriteString(formatted + "\n")

		case LogLevelWarn:
			os.Stderr.WriteString(color.RenderString(color.Yellow.Code(), formatted) + "\n")

		case LogLevelError, LogLevelFatal:
			os.Stderr.WriteString(color.RenderString(color.Red.Code(), formatted) + "\n")
		}
	}

	if (n.conf.LogDestinations&LogDestinationRosout) != 0 && n.rosoutPublisher != nil {
		n.rosoutPublisher.Write(&rosgraph_msgs.Log{
			Header: std_msgs.Header{
				Stamp: now,
			},
			Level: func() int8 {
				switch level {
				case LogLevelDebug:
					return rosgraph_msgs.Log_DEBUG
				case LogLevelInfo:
					return rosgraph_msgs.Log_INFO
				case LogLevelWarn:
					return rosgraph_msgs.Log_WARN
				case LogLevelError:
					return rosgraph_msgs.Log_ERROR
				}
				return rosgraph_msgs.Log_FATAL
			}(),
			Name: n.absoluteName(),
			Msg:  msg,
		})
	}

	if (n.conf.LogDestinations&LogDestinationCallback) != 0 && n.conf.OnLog != nil {
		n.conf.OnLog(level, msg)
	}
}
