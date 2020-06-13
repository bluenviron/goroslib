package goroslib

import (
	"regexp"
	"strconv"
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

func TestPublisherWriteToGoUdp(t *testing.T) {
	sent := &std_msgs.Int64MultiArray{}

	for i := int64(1); i <= 400; i++ {
		sent.Data = append(sent.Data, i)
	}

	recv := func() *std_msgs.Int64MultiArray {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		ns, err := NewNode(NodeConf{
			Name:       "/goroslibsub",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer ns.Close()

		recv := make(chan *std_msgs.Int64MultiArray)

		sub, err := NewSubscriber(SubscriberConf{
			Node:  ns,
			Topic: "/test_pub",
			Callback: func(msg *std_msgs.Int64MultiArray) {
				recv <- msg
			},
			Protocol: UDP,
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
			Msg:   &std_msgs.Int64MultiArray{},
		})
		require.NoError(t, err)
		defer pub.Close()

		time.Sleep(1 * time.Second)

		pub.Write(sent)

		return <-recv
	}()

	require.Equal(t, sent, recv)
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

func TestPublisherWriteToCppUdp(t *testing.T) {
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
			Msg:   &std_msgs.Int64MultiArray{},
		})
		require.NoError(t, err)
		defer pub.Close()

		rt, err := newContainer("node-sub-udp", m.Ip())
		require.NoError(t, err)

		time.Sleep(1 * time.Second)

		msg := &std_msgs.Int64MultiArray{}

		for i := int64(1); i <= 400; i++ {
			msg.Data = append(msg.Data, i)
		}

		pub.Write(msg)

		return rt.waitOutput()
	}()

	expected := "400 "
	for i := 1; i <= 400; i++ {
		expected += strconv.FormatInt(int64(i), 10) + " "
	}
	expected += "\n"

	require.Equal(t, expected, recv)
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

func TestPublisherWriteToRostopicHz(t *testing.T) {
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

		ticker := time.NewTicker(200 * time.Millisecond)
		defer ticker.Stop()

		go func() {
			for range ticker.C {
				pub.Write(&std_msgs.Float64{Data: 22.5})
			}
		}()

		rt, err := newContainer("rostopic-hz", m.Ip())
		require.NoError(t, err)

		return rt.waitOutput()
	}()

	re := regexp.MustCompile("^subscribed to \\[/test_pub\\]\naverage rate: (.+?)\nmin: (.+?)s max: (.+?)s std dev: (.+?)s window: [0-9]\n$")
	matches := re.FindStringSubmatch(recv)
	if matches == nil {
		t.Errorf("string does not match. String:\n%s", recv)
		return
	}

	v, _ := strconv.ParseFloat(matches[1], 64)
	require.Greater(t, v, 4.9)
	require.Less(t, v, 5.1)

	v, _ = strconv.ParseFloat(matches[2], 64)
	require.GreaterOrEqual(t, v, 0.0)
	require.LessOrEqual(t, v, 0.2)
}
