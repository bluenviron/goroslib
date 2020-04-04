package goroslib

import (
	"os/exec"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
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

func newRostopicEcho(masterIp string) (string, error) {
	exec.Command("docker", "kill", "goroslib-test-rostopic-echo").Run()
	exec.Command("docker", "wait", "goroslib-test-rostopic-echo").Run()
	exec.Command("docker", "rm", "goroslib-test-rostopic-echo").Run()

	cmd := []string{"docker", "run", "--rm", "--name=goroslib-test-rostopic-echo"}
	cmd = append(cmd, "-e", "MASTER_IP="+masterIp)
	cmd = append(cmd, "goroslib-test-rostopic-echo")
	c := exec.Command(cmd[0], cmd[1:]...)
	out, err := c.Output()
	if err != nil {
		return "", err
	}

	return string(out), nil
}

type Float64 struct {
	msgs.Package `ros:"std_msgs"`
	Data         msgs.Float64
}

func TestPublisherRostopicEcho(t *testing.T) {
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
		Msg:   &Float64{},
	})
	require.NoError(t, err)
	defer pub.Close()

	terminate := make(chan int)
	done := make(chan int)

	go func() {
		defer func() { done <- 1 }()

		t := time.NewTicker(1 * time.Second)
		defer t.Stop()

		for {
			select {
			case <-t.C:
				pub.Write(&Float64{Data: 34.5})

			case <-terminate:
				return
			}
		}
	}()
	defer func() {
		terminate <- 1
		<-done
	}()

	out, err := newRostopicEcho(m.Ip())
	require.NoError(t, err)
	require.Equal(t, "data: 34.5\n---\n", out)
}
