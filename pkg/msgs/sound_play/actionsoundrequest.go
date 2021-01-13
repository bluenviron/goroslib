package sound_play //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type SoundRequestActionGoal struct { //nolint:golint
	SoundRequest SoundRequest //nolint:golint
}

type SoundRequestActionResult struct { //nolint:golint
	Playing bool      //nolint:golint
	Stamp   time.Time //nolint:golint
}

type SoundRequestActionFeedback struct { //nolint:golint
	Playing bool      //nolint:golint
	Stamp   time.Time //nolint:golint
}

type SoundRequestAction struct { //nolint:golint
	msg.Package `ros:"sound_play"`
	SoundRequestActionGoal
	SoundRequestActionResult
	SoundRequestActionFeedback
}
