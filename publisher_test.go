package goroslib

import (
	"regexp"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

func TestPublisherRegister(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &TestMessage{},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	topics, err := n.MasterGetTopics()
	require.NoError(t, err)

	topic, ok := topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Publishers["/myns/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	pub.Close()
	time.Sleep(1 * time.Second)

	topics, err = n.MasterGetTopics()
	require.NoError(t, err)

	topic, ok = topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Publishers["/myns/goroslib"]
	require.Equal(t, false, ok)
}

func TestPublisherWriteAfterSub(t *testing.T) {
	sent := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	for _, sub := range []string{
		"cpp",
		"go",
	} {
		t.Run(sub, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			var subc *container
			var recv chan *TestMessage

			switch sub {
			case "cpp":
				var err error
				subc, err = newContainer("node-sub", m.IP())
				require.NoError(t, err)

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslibsub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				recv = make(chan *TestMessage)
				sub, err := NewSubscriber(SubscriberConf{
					Node:  ns,
					Topic: "test_topic",
					Callback: func(msg *TestMessage) {
						recv <- msg
					},
				})
				require.NoError(t, err)
				defer sub.Close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			pub, err := NewPublisher(PublisherConf{
				Node:  n,
				Topic: "test_topic",
				Msg:   &TestMessage{},
			})
			require.NoError(t, err)
			defer pub.Close()

			time.Sleep(1 * time.Second)

			pub.Write(sent)

			switch sub {
			case "cpp":
				require.Equal(t, "1 other test 5776731014620\n", subc.waitOutput())

			case "go":
				require.Equal(t, sent, <-recv)
			}
		})
	}
}

func TestPublisherWriteBeforeSubNoLatch(t *testing.T) {
	expected1 := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	expected2 := &std_msgs.Float64{Data: 34.5}

	for _, sub := range []string{
		"cpp",
		"go",
		"rostopic echo",
	} {
		t.Run(sub, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			var expected interface{}
			switch sub {
			case "cpp":
				expected = expected1
			case "go":
				expected = expected1
			case "rostopic echo":
				expected = expected2
			}

			pub, err := NewPublisher(PublisherConf{
				Node:  n,
				Topic: "test_topic",
				Msg:   expected,
			})
			require.NoError(t, err)
			defer pub.Close()

			var subc *container
			var recv chan *TestMessage

			switch sub {
			case "cpp":
				var err error
				subc, err = newContainer("node-sub", m.IP())
				require.NoError(t, err)

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslibsub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				recv = make(chan *TestMessage)

				sub, err := NewSubscriber(SubscriberConf{
					Node:  ns,
					Topic: "test_topic",
					Callback: func(msg *TestMessage) {
						recv <- msg
					},
				})
				require.NoError(t, err)
				defer sub.Close()

			case "rostopic echo":
				var err error
				subc, err = newContainer("rostopic-echo", m.IP())
				require.NoError(t, err)
			}

			time.Sleep(1 * time.Second)

			pub.Write(expected)

			switch sub {
			case "cpp":
				require.Equal(t, "1 other test 5776731014620\n", subc.waitOutput())

			case "go":
				require.Equal(t, expected1, <-recv)

			case "rostopic echo":
				require.Equal(t, "data: 34.5\n---\n", subc.waitOutput())
			}
		})
	}
}

func TestPublisherWriteBeforeSubLatch(t *testing.T) {
	expected1 := &TestMessage{
		A: 1,
		B: []TestParent{
			{
				A: "other test",
				B: time.Unix(1500, 1345).UTC(),
			},
		},
	}

	expected2 := &std_msgs.Float64{Data: 45.5}

	for _, sub := range []string{
		"cpp",
		"go",
		"rostopic echo",
	} {
		t.Run(sub, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			var expected interface{}
			switch sub {
			case "cpp":
				expected = expected1
			case "go":
				expected = expected1
			case "rostopic echo":
				expected = expected2
			}

			pub, err := NewPublisher(PublisherConf{
				Node:  n,
				Topic: "test_topic",
				Msg:   expected,
				Latch: true,
			})
			require.NoError(t, err)
			defer pub.Close()

			pub.Write(expected)

			ns, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslibsub",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer ns.Close()

			switch sub {
			case "cpp":
				subc, err := newContainer("node-sub", m.IP())
				require.NoError(t, err)
				require.Equal(t, "1 other test 5776731014620\n", subc.waitOutput())

			case "go":
				recv := make(chan *TestMessage)

				sub, err := NewSubscriber(SubscriberConf{
					Node:  ns,
					Topic: "test_topic",
					Callback: func(msg *TestMessage) {
						recv <- msg
					},
				})
				require.NoError(t, err)
				defer sub.Close()

				require.Equal(t, expected, <-recv)

			case "rostopic echo":
				subc, err := newContainer("rostopic-echo", m.IP())
				require.NoError(t, err)
				require.Equal(t, "data: 45.5\n---\n", subc.waitOutput())
			}
		})
	}
}

func TestPublisherWriteUdp(t *testing.T) {
	sent := &std_msgs.Int64MultiArray{}
	for i := int64(1); i <= 400; i++ {
		sent.Data = append(sent.Data, i)
	}

	for _, sub := range []string{
		"cpp",
		"go",
	} {
		t.Run(sub, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			ns, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslibsub",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer ns.Close()

			var subc *container
			var recv chan *std_msgs.Int64MultiArray

			switch sub {
			case "cpp":
				var err error
				subc, err = newContainer("node-sub-udp", m.IP())
				require.NoError(t, err)

			case "go":
				recv = make(chan *std_msgs.Int64MultiArray)

				sub, err := NewSubscriber(SubscriberConf{
					Node:  ns,
					Topic: "test_topic",
					Callback: func(msg *std_msgs.Int64MultiArray) {
						recv <- msg
					},
					Protocol: UDP,
				})
				require.NoError(t, err)
				defer sub.Close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			pub, err := NewPublisher(PublisherConf{
				Node:  n,
				Topic: "test_topic",
				Msg:   &std_msgs.Int64MultiArray{},
			})
			require.NoError(t, err)
			defer pub.Close()

			time.Sleep(1 * time.Second)

			pub.Write(sent)

			switch sub {
			case "cpp":
				expected := "400 "
				for i := 1; i <= 400; i++ {
					expected += strconv.FormatInt(int64(i), 10) + " "
				}
				expected += "\n"
				require.Equal(t, expected, subc.waitOutput())

			case "go":
				require.Equal(t, sent, <-recv)
			}
		})
	}
}

func TestPublisherRostopicHz(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &std_msgs.Float64{},
	})
	require.NoError(t, err)
	defer pub.Close()

	pubTerminate := make(chan struct{})
	defer close(pubTerminate)

	pubDone := make(chan struct{})
	go func() {
		defer close(pubDone)

		t := time.NewTicker(200 * time.Millisecond)
		defer t.Stop()

		for {
			select {
			case <-t.C:
				pub.Write(&std_msgs.Float64{Data: 22.5})

			case <-pubTerminate:
				return
			}
		}
	}()

	rt, err := newContainer("rostopic-hz", m.IP())
	require.NoError(t, err)

	recv := rt.waitOutput()
	lines := strings.Split(recv, "\n")
	line := lines[len(lines)-2]
	line = strings.TrimSpace(line)

	ma := regexp.MustCompile("^min: (.+?)s max: (.+?)s std dev: (.+?)s window: [0-9]$").
		FindStringSubmatch(line)
	if ma == nil {
		t.Errorf("line does not match (%s)", line)
		return
	}

	v, _ := strconv.ParseFloat(ma[1], 64)
	require.Greater(t, v, 0.192)
	require.Less(t, v, 0.208)

	v, _ = strconv.ParseFloat(ma[2], 64)
	require.Greater(t, v, 0.192)
	require.Less(t, v, 0.208)
}
