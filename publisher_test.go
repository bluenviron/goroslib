package goroslib

import (
	"bytes"
	"net"
	"regexp"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/apislave"
	"github.com/aler9/goroslib/pkg/msgproc"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

func TestPublisherOpen(t *testing.T) {
	m := newContainerMaster(t)
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

func TestPublisherOpenErrors(t *testing.T) {
	_, err := NewPublisher(PublisherConf{})
	require.Error(t, err)

	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib-server",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	_, err = NewPublisher(PublisherConf{
		Node: n,
	})
	require.Error(t, err)

	_, err = NewPublisher(PublisherConf{
		Node:  n,
		Topic: "mytopic",
	})
	require.Error(t, err)

	_, err = NewPublisher(PublisherConf{
		Node:  n,
		Topic: "mytopic",
		Msg:   123,
	})
	require.Error(t, err)
}

func TestPublisherWrite(t *testing.T) {
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
			m := newContainerMaster(t)
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
				subc = newContainer(t, "node-sub", m.IP())

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslibsub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				recv = make(chan *TestMessage)

				uris, err := ns.apiMasterClient.RegisterSubscriber(
					ns.absoluteTopicName("test_topic"),
					"goroslib/TestMessage",
					ns.apiSlaveServer.URL())
				require.NoError(t, err)

				addr, err := urlToAddress(uris[0])
				require.NoError(t, err)

				xcs := apislave.NewClient(addr, ns.absoluteName())

				proto, err := xcs.RequestTopic(
					ns.absoluteTopicName("test_topic"),
					[][]interface{}{{"TCPROS"}})
				require.NoError(t, err)
				require.Equal(t, "TCPROS", proto[0])

				addr = net.JoinHostPort(proto[1].(string),
					strconv.FormatInt(int64(proto[2].(int)), 10))

				conn, err := prototcp.NewClient(addr)
				require.NoError(t, err)
				defer conn.Close()

				msgMd5, err := msgproc.MD5(&TestMessage{})
				require.NoError(t, err)

				go func() {
					err := conn.WriteHeader(&prototcp.HeaderSubscriber{
						Callerid:   ns.absoluteName(),
						Md5sum:     msgMd5,
						Topic:      ns.absoluteTopicName("test_topic"),
						Type:       "goroslib/TestMessage",
						TcpNodelay: 1,
					})
					require.NoError(t, err)

					raw, err := conn.ReadHeaderRaw()
					require.NoError(t, err)

					var outHeader prototcp.HeaderPublisher
					err = protocommon.HeaderDecode(raw, &outHeader)
					require.NoError(t, err)

					require.Equal(t, "/myns/test_topic", outHeader.Topic)
					require.Equal(t, msgMd5, outHeader.Md5sum)

					var msg TestMessage
					err = conn.ReadMessage(&msg)
					require.NoError(t, err)
					recv <- &msg
				}()

			case "rostopic echo":
				subc = newContainer(t, "rostopic-echo", m.IP())
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

func TestPublisherWriteLatch(t *testing.T) {
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
			m := newContainerMaster(t)
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

			switch sub {
			case "cpp":
				subc := newContainer(t, "node-sub", m.IP())
				require.Equal(t, "1 other test 5776731014620\n", subc.waitOutput())

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslibsub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				recv := make(chan *TestMessage)

				uris, err := ns.apiMasterClient.RegisterSubscriber(
					ns.absoluteTopicName("test_topic"),
					"goroslib/TestMessage",
					ns.apiSlaveServer.URL())
				require.NoError(t, err)

				addr, err := urlToAddress(uris[0])
				require.NoError(t, err)

				xcs := apislave.NewClient(addr, ns.absoluteName())

				proto, err := xcs.RequestTopic(
					ns.absoluteTopicName("test_topic"),
					[][]interface{}{{"TCPROS"}})
				require.NoError(t, err)
				require.Equal(t, "TCPROS", proto[0])

				addr = net.JoinHostPort(proto[1].(string),
					strconv.FormatInt(int64(proto[2].(int)), 10))

				conn, err := prototcp.NewClient(addr)
				require.NoError(t, err)
				defer conn.Close()

				msgMd5, err := msgproc.MD5(&TestMessage{})
				require.NoError(t, err)

				go func() {
					err := conn.WriteHeader(&prototcp.HeaderSubscriber{
						Callerid:   ns.absoluteName(),
						Md5sum:     msgMd5,
						Topic:      ns.absoluteTopicName("test_topic"),
						Type:       "goroslib/TestMessage",
						TcpNodelay: 1,
					})
					require.NoError(t, err)

					raw, err := conn.ReadHeaderRaw()
					require.NoError(t, err)

					var outHeader prototcp.HeaderPublisher
					err = protocommon.HeaderDecode(raw, &outHeader)
					require.NoError(t, err)

					require.Equal(t, "/myns/test_topic", outHeader.Topic)
					require.Equal(t, msgMd5, outHeader.Md5sum)

					var msg TestMessage
					err = conn.ReadMessage(&msg)
					require.NoError(t, err)
					recv <- &msg
				}()

				require.Equal(t, expected, <-recv)

			case "rostopic echo":
				subc := newContainer(t, "rostopic-echo", m.IP())
				require.Equal(t, "data: 45.5\n---\n", subc.waitOutput())
			}
		})
	}
}

func TestPublisherWriteUDP(t *testing.T) {
	sent := &std_msgs.Int64MultiArray{}
	for i := int64(1); i <= 400; i++ {
		sent.Data = append(sent.Data, i)
	}

	for _, sub := range []string{
		"cpp",
		"go",
	} {
		t.Run(sub, func(t *testing.T) {
			m := newContainerMaster(t)
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
				Msg:   &std_msgs.Int64MultiArray{},
			})
			require.NoError(t, err)
			defer pub.Close()

			var subc *container
			var recv chan *std_msgs.Int64MultiArray

			switch sub {
			case "cpp":
				subc = newContainer(t, "node-sub-udp", m.IP())

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslibsub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				recv = make(chan *std_msgs.Int64MultiArray)

				uris, err := ns.apiMasterClient.RegisterSubscriber(
					ns.absoluteTopicName("test_topic"),
					"std_msgs/Int64MultiArray",
					ns.apiSlaveServer.URL())
				require.NoError(t, err)

				addr, err := urlToAddress(uris[0])
				require.NoError(t, err)

				xcs := apislave.NewClient(addr, ns.absoluteName())

				msgMd5, err := msgproc.MD5(&std_msgs.Int64MultiArray{})
				require.NoError(t, err)

				udprosServer, err := protoudp.NewServer(ns.nodeAddr.IP.String() + ":3334")
				require.NoError(t, err)

				proto, err := xcs.RequestTopic(
					ns.absoluteTopicName("test_topic"),
					[][]interface{}{{
						"UDPROS",
						func() []byte {
							var buf bytes.Buffer
							protocommon.HeaderEncode(&buf, &protoudp.HeaderSubscriber{
								Callerid: ns.absoluteName(),
								Md5sum:   msgMd5,
								Topic:    ns.absoluteTopicName("test_topic"),
								Type:     "std_msgs/Int64MultiArray",
							})
							return buf.Bytes()[4:]
						}(),
						n.nodeAddr.IP.String(),
						udprosServer.Port(),
						1500,
					}})
				require.NoError(t, err)
				require.Equal(t, "UDPROS", proto[0])

				go func() {
					var buf []byte

					frame, _, err := udprosServer.ReadFrame()
					require.NoError(t, err)
					require.Equal(t, protoudp.Data0, frame.Opcode)
					buf = append(buf, frame.Payload...)

					frame, _, err = udprosServer.ReadFrame()
					require.NoError(t, err)
					require.Equal(t, protoudp.DataN, frame.Opcode)
					buf = append(buf, frame.Payload...)

					frame, _, err = udprosServer.ReadFrame()
					require.NoError(t, err)
					require.Equal(t, protoudp.DataN, frame.Opcode)
					buf = append(buf, frame.Payload...)

					var msg std_msgs.Int64MultiArray
					err = protocommon.MessageDecode(bytes.NewReader(buf), &msg)
					require.NoError(t, err)
					recv <- &msg
				}()
			}

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
	m := newContainerMaster(t)
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

	rt := newContainer(t, "rostopic-hz", m.IP())

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

func TestPublisherAcceptErrors(t *testing.T) {
	t.Run("invalid header", func(t *testing.T) {
		m := newContainerMaster(t)
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

		ns, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslibsub",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer ns.Close()

		uris, err := ns.apiMasterClient.RegisterSubscriber(
			ns.absoluteTopicName("test_topic"),
			"std_msgs/Int64MultiArray",
			ns.apiSlaveServer.URL())
		require.NoError(t, err)

		addr, err := urlToAddress(uris[0])
		require.NoError(t, err)

		xcs := apislave.NewClient(addr, ns.absoluteName())

		proto, err := xcs.RequestTopic(
			ns.absoluteTopicName("test_topic"),
			[][]interface{}{{"TCPROS"}})
		require.NoError(t, err)

		addr = net.JoinHostPort(proto[1].(string),
			strconv.FormatInt(int64(proto[2].(int)), 10))

		conn, err := prototcp.NewClient(addr)
		require.NoError(t, err)
		defer conn.Close()

		conn.NetConn().Write([]byte{0x01})

		buf := make([]byte, 1024)
		_, err = conn.NetConn().Read(buf)
		require.Error(t, err)
	})

	t.Run("invalid header 2", func(t *testing.T) {
		m := newContainerMaster(t)
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

		ns, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslibsub",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer ns.Close()

		uris, err := ns.apiMasterClient.RegisterSubscriber(
			ns.absoluteTopicName("test_topic"),
			"std_msgs/Int64MultiArray",
			ns.apiSlaveServer.URL())
		require.NoError(t, err)

		addr, err := urlToAddress(uris[0])
		require.NoError(t, err)

		xcs := apislave.NewClient(addr, ns.absoluteName())

		proto, err := xcs.RequestTopic(
			ns.absoluteTopicName("test_topic"),
			[][]interface{}{{"TCPROS"}})
		require.NoError(t, err)

		addr = net.JoinHostPort(proto[1].(string),
			strconv.FormatInt(int64(proto[2].(int)), 10))

		conn, err := prototcp.NewClient(addr)
		require.NoError(t, err)
		defer conn.Close()

		type HeaderInvalid struct {
			Topic string
		}

		conn.WriteHeader(&HeaderInvalid{Topic: "invalid"})

		buf := make([]byte, 1024)
		_, err = conn.NetConn().Read(buf)
		require.Error(t, err)
	})
}
