package goroslib

import (
	"time"
)

// TimeNow returns the current time (real or simulated).
func (n *Node) TimeNow() time.Time {
	if !n.conf.UseSimTime {
		return time.Now()
	}

	n.simtimeMutex.RLock()
	defer n.simtimeMutex.RUnlock()

	return n.simtimeValue
}

// TimeSleep sleeps for the given amount of time (real or simulated).
func (n *Node) TimeSleep(d time.Duration) {
	if !n.conf.UseSimTime {
		time.Sleep(d)
		return
	}

	done := make(chan struct{})
	func() {
		n.simtimeMutex.Lock()
		defer n.simtimeMutex.Unlock()

		n.simtimeSleeps = append(n.simtimeSleeps,
			&simtimeSleep{n.simtimeValue.Add(d), done})
	}()
	<-done
}

// NodeRate allows to sleep with a given period.
type NodeRate struct {
	n         *Node
	d         time.Duration
	lastSleep *simtimeSleep
}

// Sleep sleeps with a given period.
func (nr *NodeRate) Sleep() {
	if !nr.n.conf.UseSimTime {
		now := time.Now()

		if nr.lastSleep == nil {
			nr.lastSleep = &simtimeSleep{now.Add(nr.d), nil}
		} else {
			nr.lastSleep = &simtimeSleep{nr.lastSleep.value.Add(nr.d), nil}
		}

		time.Sleep(nr.lastSleep.value.Sub(now))
		return
	}

	done := make(chan struct{})
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
	<-done
}

// TimeRate returns an object that can be used to sleep periodically.
func (n *Node) TimeRate(d time.Duration) *NodeRate {
	return &NodeRate{
		n: n,
		d: d,
	}
}
