package ackermann_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AckermannDrive struct { //nolint:golint
	msg.Package           `ros:"ackermann_msgs"`
	SteeringAngle         float32 //nolint:golint
	SteeringAngleVelocity float32 //nolint:golint
	Speed                 float32 //nolint:golint
	Acceleration          float32 //nolint:golint
	Jerk                  float32 //nolint:golint
}
