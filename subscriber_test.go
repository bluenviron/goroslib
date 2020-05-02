package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type TestParent struct {
	A msgs.String
	B msgs.Time
	C msgs.Bool
	D msgs.Byte
	E msgs.Char
	F msgs.Duration
}

type TestMessage struct {
	A msgs.Uint8
	B []TestParent
	C [2]TestParent
	D [2]msgs.Uint32
}

func TestSubscriberRegister(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	sub, err := NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "/test_pub",
		Callback: func(msg *TestMessage) {
		},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	topics, err := n.GetTopics()
	require.NoError(t, err)

	topic, ok := topics["/test_pub"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	sub.Close()
	time.Sleep(1 * time.Second)

	topics, err = n.GetTopics()
	require.NoError(t, err)

	topic, ok = topics["/test_pub"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/goroslib"]
	require.Equal(t, false, ok)
}

func TestSubscriberReadAfterPub(t *testing.T) {
	recv := func() *TestMessage {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		p, err := newContainer("node-pub", m.Ip())
		require.NoError(t, err)
		defer p.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		chanRecv := make(chan *TestMessage, 10)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  n,
			Topic: "/test_pub",
			Callback: func(msg *TestMessage) {
				chanRecv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		return <-chanRecv
	}()

	expected := TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
				C: true,
				D: 27,
				E: 23,
				F: 2345500 * time.Millisecond,
			},
		},
		C: [2]TestParent{
			{
				A: "AA",
			},
			{
				A: "BB",
			},
		},
		D: [2]msgs.Uint32{222, 333},
	}
	require.Equal(t, &expected, recv)
}

func TestSubscriberReadBeforePub(t *testing.T) {
	recv := func() *TestMessage {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		chanRecv := make(chan *TestMessage, 10)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  n,
			Topic: "/test_pub",
			Callback: func(msg *TestMessage) {
				chanRecv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		p, err := newContainer("node-pub", m.Ip())
		require.NoError(t, err)
		defer p.close()

		return <-chanRecv
	}()

	expected := TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
				C: true,
				D: 27,
				E: 23,
				F: 2345500 * time.Millisecond,
			},
		},
		C: [2]TestParent{
			{
				A: "AA",
			},
			{
				A: "BB",
			},
		},
		D: [2]msgs.Uint32{222, 333},
	}
	require.Equal(t, &expected, recv)
}

func TestSubscriberRostopicPub(t *testing.T) {
	recv := func() *sensor_msgs.Imu {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		p, err := newContainer("rostopic-pub", m.Ip())
		require.NoError(t, err)
		defer p.close()

		chanRecv := make(chan *sensor_msgs.Imu, 10)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  n,
			Topic: "/test_pub",
			Callback: func(msg *sensor_msgs.Imu) {
				chanRecv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		return <-chanRecv
	}()

	expected := &sensor_msgs.Imu{
		Header: std_msgs.Header{
			Seq: 1,
		},
		OrientationCovariance:        [9]msgs.Float64{0, 0, 0, 0, 0.2, 0, 0, 0, 0},
		AngularVelocityCovariance:    [9]msgs.Float64{0, 0, 15, 0, 0, 0, 0, 0, 0},
		LinearAccelerationCovariance: [9]msgs.Float64{0, 0, 0, 0, 0, 0, 0, 0, 13.7},
	}
	require.Equal(t, expected, recv)
}
