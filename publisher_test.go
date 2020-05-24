package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs/std_msgs"
)

func TestPublisherRegister(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "/test_pub",
		Msg:   &TestMessage{},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	topics, err := n.GetTopics()
	require.NoError(t, err)

	topic, ok := topics["/test_pub"]
	require.Equal(t, true, ok)

	_, ok = topic.Publishers["/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	pub.Close()
	time.Sleep(1 * time.Second)

	topics, err = n.GetTopics()
	require.NoError(t, err)

	topic, ok = topics["/test_pub"]
	require.Equal(t, true, ok)

	_, ok = topic.Publishers["/goroslib"]
	require.Equal(t, false, ok)
}

func TestPublisherWriteToGoAfterSubNoLatch(t *testing.T) {
	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	recv := func() *TestMessage {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		ns, err := NewNode(NodeConf{
			Name:       "/goroslibsub",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer ns.Close()

		recv := make(chan *TestMessage)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  ns,
			Topic: "/test_pub",
			Callback: func(msg *TestMessage) {
				recv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &TestMessage{},
		})
		require.NoError(t, err)
		defer pub.Close()

		time.Sleep(1 * time.Second)

		pub.Write(sent)

		return <-recv
	}()

	require.Equal(t, sent, recv)
}

func TestPublisherWriteToGoBeforeSubNoLatch(t *testing.T) {
	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

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

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &TestMessage{},
		})
		require.NoError(t, err)
		defer pub.Close()

		ns, err := NewNode(NodeConf{
			Name:       "/goroslibsub",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer ns.Close()

		recv := make(chan *TestMessage)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  ns,
			Topic: "/test_pub",
			Callback: func(msg *TestMessage) {
				recv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		time.Sleep(1 * time.Second)

		pub.Write(sent)

		return <-recv
	}()

	require.Equal(t, sent, recv)
}

func TestPublisherWriteToGoLatch(t *testing.T) {
	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

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

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &TestMessage{},
			Latch: true,
		})
		require.NoError(t, err)
		defer pub.Close()

		pub.Write(sent)

		ns, err := NewNode(NodeConf{
			Name:       "/goroslibsub",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer ns.Close()

		recv := make(chan *TestMessage)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  ns,
			Topic: "/test_pub",
			Callback: func(msg *TestMessage) {
				recv <- msg
			},
		})
		require.NoError(t, err)
		defer sub.Close()

		return <-recv
	}()

	require.Equal(t, sent, recv)
}

func TestPublisherWriteToRostopicEchoNoLatch(t *testing.T) {
	recv := func() string {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &std_msgs.Float64{},
		})
		require.NoError(t, err)
		defer pub.Close()

		rt, err := newContainer("rostopic-echo", m.Ip())
		require.NoError(t, err)

		time.Sleep(1 * time.Second)

		pub.Write(&std_msgs.Float64{Data: 34.5})

		return rt.waitOutput()
	}()

	require.Equal(t, "data: 34.5\n---\n", recv)
}

func TestPublisherWriteToCppLatch(t *testing.T) {
	recv := func() string {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &TestMessage{},
			Latch: true,
		})
		require.NoError(t, err)
		defer pub.Close()

		pub.Write(&TestMessage{
			A: 1,
			B: []TestParent{
				{
					A: "other test",
					B: time.Unix(1500, 1345).UTC(),
				},
			},
		})

		rt, err := newContainer("node-sub", m.Ip())
		require.NoError(t, err)

		return rt.waitOutput()
	}()

	require.Equal(t, "1 other test 5776731014620\n", recv)
}

func TestPublisherWriteToRostopicEchoLatch(t *testing.T) {
	recv := func() string {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &std_msgs.Float64{},
			Latch: true,
		})
		require.NoError(t, err)
		defer pub.Close()

		pub.Write(&std_msgs.Float64{Data: 45.5})

		rt, err := newContainer("rostopic-echo", m.Ip())
		require.NoError(t, err)

		return rt.waitOutput()
	}()

	require.Equal(t, "data: 45.5\n---\n", recv)
}
