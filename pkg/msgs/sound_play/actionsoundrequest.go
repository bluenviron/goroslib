//nolint:golint,lll
package sound_play

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
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
