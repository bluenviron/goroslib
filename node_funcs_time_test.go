package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/rosgraph_msgs"
)

func enableSimTime(m *containerMaster) error {
	ns, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib_ns",
		MasterAddress: m.IP() + ":11311",
	})
	if err != nil {
		return err
	}
	defer ns.Close()

	return ns.ParamSetBool("/use_sim_time", true)
}

func TestNodeTimeNow(t *testing.T) {
	t.Run("real", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		exp := time.Now()
		val := n.TimeNow()

		if val.Before(exp.Add(-10 * time.Millisecond)) {
			t.Errorf("should not happen")
		} else if exp.Add(10 * time.Millisecond).Before(val) {
			t.Errorf("should not happen")
		}
	})

	t.Run("simulated", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		err := enableSimTime(m)
		require.NoError(t, err)

		cs, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib_cs",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer cs.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  cs,
			Topic: "/clock",
			Msg:   &rosgraph_msgs.Clock{},
		})
		require.NoError(t, err)
		defer pub.Close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		time.Sleep(100 * time.Millisecond)

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(5, 0),
		})

		time.Sleep(100 * time.Millisecond)

		zero := time.Unix(0, 0)
		require.Equal(t, zero.Add(5*time.Second).Unix(), n.TimeNow().Unix())
	})
}

func TestNodeTimeSleep(t *testing.T) {
	t.Run("real", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		sleepDone := make(chan struct{})
		go func() {
			defer close(sleepDone)
			n.TimeSleep(1 * time.Second)
		}()

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		select {
		case <-time.After(1 * time.Second):
			t.Errorf("should not happen")
		case <-sleepDone:
		}
	})

	t.Run("simulated, before first clock message", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		err := enableSimTime(m)
		require.NoError(t, err)

		cs, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib_cs",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer cs.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  cs,
			Topic: "/clock",
			Msg:   &rosgraph_msgs.Clock{},
		})
		require.NoError(t, err)
		defer pub.Close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		sleepDone := make(chan struct{})
		go func() {
			defer close(sleepDone)
			n.TimeSleep(5 * time.Second)
		}()

		time.Sleep(100 * time.Millisecond)

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(5, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(10, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}
	})

	t.Run("simulated, after first clock message", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		err := enableSimTime(m)
		require.NoError(t, err)

		cs, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib_cs",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer cs.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  cs,
			Topic: "/clock",
			Msg:   &rosgraph_msgs.Clock{},
		})
		require.NoError(t, err)
		defer pub.Close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		time.Sleep(100 * time.Millisecond)

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(5, 0),
		})

		sleepDone := make(chan struct{})
		go func() {
			defer close(sleepDone)
			n.TimeSleep(5 * time.Second)
		}()

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(10, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}
	})
}

func TestNodeTimeRate(t *testing.T) {
	t.Run("real", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		r := n.TimeRate(1 * time.Second)

		for i := 0; i < 2; i++ {
			sleepDone := make(chan struct{})
			go func() {
				defer close(sleepDone)
				r.Sleep()
			}()

			select {
			case <-time.After(100 * time.Millisecond):
			case <-sleepDone:
				t.Errorf("should not happen")
			}

			select {
			case <-time.After(1 * time.Second):
				t.Errorf("should not happen")
			case <-sleepDone:
			}
		}
	})

	t.Run("simulated, before first clock message", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		err := enableSimTime(m)
		require.NoError(t, err)

		cs, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib_cs",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer cs.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  cs,
			Topic: "/clock",
			Msg:   &rosgraph_msgs.Clock{},
		})
		require.NoError(t, err)
		defer pub.Close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		r := n.TimeRate(5 * time.Second)

		sleepDone := make(chan struct{})
		go func() {
			defer close(sleepDone)
			r.Sleep()
		}()

		time.Sleep(100 * time.Millisecond)

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(5, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(10, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}

		sleepDone = make(chan struct{})
		go func() {
			defer close(sleepDone)
			r.Sleep()
		}()

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(15, 0),
		})

		time.Sleep(100 * time.Millisecond)

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}
	})

	t.Run("simulated, after first clock message", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		err := enableSimTime(m)
		require.NoError(t, err)

		cs, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib_cs",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer cs.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  cs,
			Topic: "/clock",
			Msg:   &rosgraph_msgs.Clock{},
		})
		require.NoError(t, err)
		defer pub.Close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		time.Sleep(100 * time.Millisecond)

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(5, 0),
		})

		time.Sleep(100 * time.Millisecond)

		r := n.TimeRate(5 * time.Second)

		sleepDone := make(chan struct{})
		go func() {
			defer close(sleepDone)
			r.Sleep()
		}()

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(10, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}

		sleepDone = make(chan struct{})
		go func() {
			defer close(sleepDone)
			r.Sleep()
		}()

		select {
		case <-time.After(100 * time.Millisecond):
		case <-sleepDone:
			t.Errorf("should not happen")
		}

		pub.Write(&rosgraph_msgs.Clock{
			Clock: time.Unix(15, 0),
		})

		select {
		case <-time.After(100 * time.Millisecond):
			t.Errorf("should not happen")
		case <-sleepDone:
		}
	})
}
