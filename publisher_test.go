package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestPublisherRegister(t *testing.T) {
	m, err := newCntMaster()
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

func TestPublisherWriteAfterSub(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	ns, err := NewNode(NodeConf{
		Name:       "/goroslibsub",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer ns.Close()

	chanRecv := make(chan *TestMessage)

	sub, err := NewSubscriber(SubscriberConf{
		Node:  ns,
		Topic: "/test_pub",
		Callback: func(msg *TestMessage) {
			chanRecv <- msg
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

	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	time.Sleep(1 * time.Second)

	pub.Write(sent)

	recv := <-chanRecv
	require.Equal(t, sent, recv)
}

func TestPublisherWriteBeforeSub(t *testing.T) {
	m, err := newCntMaster()
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

	chanRecv := make(chan *TestMessage)

	sub, err := NewSubscriber(SubscriberConf{
		Node:  ns,
		Topic: "/test_pub",
		Callback: func(msg *TestMessage) {
			chanRecv <- msg
		},
	})
	require.NoError(t, err)
	defer sub.Close()

	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	time.Sleep(1 * time.Second)

	pub.Write(sent)

	recv := <-chanRecv
	require.Equal(t, sent, recv)
}
