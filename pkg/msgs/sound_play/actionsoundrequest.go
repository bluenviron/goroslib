//autogenerated:yes
//nolint:revive,lll
package sound_play

import (
	"time"

	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type SoundRequestActionGoal struct {
	msg.Package  `ros:"sound_play"`
	SoundRequest SoundRequest
}

type SoundRequestActionResult struct {
	msg.Package `ros:"sound_play"`
	Playing     bool
	Stamp       time.Time
}

type SoundRequestActionFeedback struct {
	msg.Package `ros:"sound_play"`
	Playing     bool
	Stamp       time.Time
}

type SoundRequestAction struct {
	msg.Package `ros:"sound_play"`
	SoundRequestActionGoal
	SoundRequestActionResult
	SoundRequestActionFeedback
}
