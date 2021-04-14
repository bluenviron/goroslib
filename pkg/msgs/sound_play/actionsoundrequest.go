//nolint:golint
package sound_play

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type SoundRequestActionGoal struct {
	SoundRequest SoundRequest
}

type SoundRequestActionResult struct {
	Playing bool
	Stamp   time.Time
}

type SoundRequestActionFeedback struct {
	Playing bool
	Stamp   time.Time
}

type SoundRequestAction struct {
	msg.Package `ros:"sound_play"`
	SoundRequestActionGoal
	SoundRequestActionResult
	SoundRequestActionFeedback
}
