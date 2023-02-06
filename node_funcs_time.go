package goroslib

import (
	"time"
)

// TimeNow returns the current time.
// It supports simulated clocks provided by ROS clock servers.
func (n *Node) TimeNow() time.Time {
	if !n.simtimeEnabled {
		return time.Now()
	}

	n.simtimeMutex.RLock()
	defer n.simtimeMutex.RUnlock()

	return n.simtimeValue
}

// TimeSleepChan returns a channel that allows to sleeps for the given amount of time.
// It supports simulated clocks provided by ROS clock servers.
func (n *Node) TimeSleepChan(d time.Duration) <-chan time.Time {
	if !n.simtimeEnabled {
		return time.After(d)
	}

	done := make(chan time.Time)
	func() {
		n.simtimeMutex.Lock()
		defer n.simtimeMutex.Unlock()

		n.simtimeSleeps = append(n.simtimeSleeps,
			&simtimeSleep{n.simtimeValue.Add(d), done})
	}()
	return done
}

// TimeSleep sleeps for the given amount of time.
// It supports simulated clocks provided by ROS clock servers.
func (n *Node) TimeSleep(d time.Duration) {
	<-n.TimeSleepChan(d)
}

// NodeRate allows to sleep with a given period.
type NodeRate struct {
	n         *Node
	d         time.Duration
	lastSleep *simtimeSleep
}

// SleepChan returns a channel that allows to sleep with a given period.
// It supports simulated clocks provided by ROS clock servers.
func (nr *NodeRate) SleepChan() <-chan time.Time {
	if !nr.n.simtimeEnabled {
		now := time.Now()

		if nr.lastSleep == nil {
			nr.lastSleep = &simtimeSleep{now.Add(nr.d), nil}
		} else {
			nr.lastSleep = &simtimeSleep{nr.lastSleep.value.Add(nr.d), nil}
		}

		return time.After(nr.lastSleep.value.Sub(now))
	}

	done := make(chan time.Time)
	func() {
		nr.n.simtimeMutex.Lock()
		defer nr.n.simtimeMutex.Unlock()

		if nr.lastSleep == nil {
			nr.lastSleep = &simtimeSleep{nr.n.simtimeValue.Add(nr.d), done}
		} else {
			nr.lastSleep = &simtimeSleep{nr.lastSleep.value.Add(nr.d), done}
		}

		nr.n.simtimeSleeps = append(nr.n.simtimeSleeps,
			nr.lastSleep)
	}()
	return done
}

// Sleep sleeps with a given period.
// It supports simulated clocks provided by ROS clock servers.
func (nr *NodeRate) Sleep() {
	<-nr.SleepChan()
}

// TimeRate returns an object that can be used to sleep periodically.
// It supports simulated clocks provided by ROS clock servers.
func (n *Node) TimeRate(d time.Duration) *NodeRate {
	return &NodeRate{
		n: n,
		d: d,
	}
}
