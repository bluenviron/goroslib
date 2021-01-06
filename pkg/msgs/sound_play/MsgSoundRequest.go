package sound_play //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	SoundRequest_BACKINGUP              int8 = 1  //nolint:golint
	SoundRequest_NEEDS_UNPLUGGING       int8 = 2  //nolint:golint
	SoundRequest_NEEDS_PLUGGING         int8 = 3  //nolint:golint
	SoundRequest_NEEDS_UNPLUGGING_BADLY int8 = 4  //nolint:golint
	SoundRequest_NEEDS_PLUGGING_BADLY   int8 = 5  //nolint:golint
	SoundRequest_ALL                    int8 = -1 //nolint:golint
	SoundRequest_PLAY_FILE              int8 = -2 //nolint:golint
	SoundRequest_SAY                    int8 = -3 //nolint:golint
	SoundRequest_PLAY_STOP              int8 = 0  //nolint:golint
	SoundRequest_PLAY_ONCE              int8 = 1  //nolint:golint
	SoundRequest_PLAY_START             int8 = 2  //nolint:golint
)

type SoundRequest struct { //nolint:golint
	msg.Package     `ros:"sound_play"`
	msg.Definitions `ros:"int8 BACKINGUP=1,int8 NEEDS_UNPLUGGING=2,int8 NEEDS_PLUGGING=3,int8 NEEDS_UNPLUGGING_BADLY=4,int8 NEEDS_PLUGGING_BADLY=5,int8 ALL=-1,int8 PLAY_FILE=-2,int8 SAY=-3,int8 PLAY_STOP=0,int8 PLAY_ONCE=1,int8 PLAY_START=2"`
	Sound           int8    //nolint:golint
	Command         int8    //nolint:golint
	Volume          float32 //nolint:golint
	Arg             string  //nolint:golint
	Arg2            string  //nolint:golint
}
