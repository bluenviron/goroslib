package goroslib

import (
	"bytes"
	"net"
	"strconv"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/apimaster"
	"github.com/aler9/goroslib/pkg/apislave"
	"github.com/aler9/goroslib/pkg/msgproc"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

type TestParent struct {
	A string
	B time.Time
	C bool
	D int8
	E uint8
	F time.Duration
}

type TestMessage struct {
	A uint8
	B []TestParent
	C [2]TestParent
	D [2]uint32
}

type testPublisher struct {
	apiSlaveServer *apislave.Server
	tcprosServer   *prototcp.Server
}

func newTestPublisher(t *testing.T, masterIP string,
	cb func(prototcp.HeaderSubscriber, *prototcp.Conn)) *testPublisher {
	masterAddr, err := net.ResolveTCPAddr("tcp", masterIP+":11311")
	require.NoError(t, err)

	host := findNodeHost(masterAddr)
	require.NotEqual(t, "", host)

	nodeAddr, err := net.ResolveTCPAddr("tcp", net.JoinHostPort(host, "0"))
	require.NoError(t, err)

	tcprosServer, err := prototcp.NewServer(nodeAddr.IP.String()+":9912", nodeAddr.IP, nodeAddr.Zone)
	require.NoError(t, err)

	go func() {
		conn, err := tcprosServer.Accept()
		require.NoError(t, err)
		defer conn.Close()

		rawHeader, err := conn.ReadHeaderRaw()
		require.NoError(t, err)

		var header prototcp.HeaderSubscriber
		err = protocommon.HeaderDecode(rawHeader, &header)
		require.NoError(t, err)

		cb(header, conn)
	}()

	apiSlaveServer, err := apislave.NewServer(nodeAddr.IP.String()+":9911", nodeAddr.IP, nodeAddr.Zone)
	require.NoError(t, err)

	go apiSlaveServer.Serve(func(req apislave.Request) apislave.Response {
		if req2, ok := req.(*apislave.RequestRequestTopic); ok {
			require.Equal(t, "/myns/goroslib", req2.CallerID)
			require.Equal(t, "/myns/test_topic", req2.Topic)
			require.Equal(t, [][]interface{}{{"TCPROS"}}, req2.Protocols)

			return apislave.ResponseRequestTopic{
				Code: 1,
				Protocol: []interface{}{
					"TCPROS",
					nodeAddr.IP.String(),
					tcprosServer.Port(),
				},
			}
		}
		return apislave.ErrorRes{}
	})

	apiMasterClient := apimaster.NewClient(masterAddr.String(), "myns/goroslib_pub")
	_, err = apiMasterClient.RegisterPublisher(
		"/myns/test_topic",
		"goroslib/TestMessage",
		apiSlaveServer.URL())
	require.NoError(t, err)

	return &testPublisher{
		apiSlaveServer: apiSlaveServer,
		tcprosServer:   tcprosServer,
	}
}

func (tp *testPublisher) close() {
	tp.apiSlaveServer.Close()
	tp.tcprosServer.Close()
}

func TestSubscriberOpen(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	sub, err := NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "test_topic",
		Callback: func(msg *TestMessage) {
		},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	topics, err := n.MasterGetTopics()
	require.NoError(t, err)

	topic, ok := topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/myns/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	sub.Close()
	time.Sleep(1 * time.Second)

	topics, err = n.MasterGetTopics()
	require.NoError(t, err)

	topic, ok = topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/myns/goroslib"]
	require.Equal(t, false, ok)
}

func TestSubscriberOpenErrors(t *testing.T) {
	_, err := NewSubscriber(SubscriberConf{})
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

	_, err = NewSubscriber(SubscriberConf{
		Node: n,
	})
	require.Error(t, err)

	_, err = NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "mytopic",
	})
	require.Error(t, err)

	_, err = NewSubscriber(SubscriberConf{
		Node:     n,
		Topic:    "mytopic",
		Callback: 123,
	})
	require.Error(t, err)
}

func TestSubscriberReadAfterPub(t *testing.T) {
	expected1 := &TestMessage{
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
		D: [2]uint32{222, 333},
	}

	expected2 := &sensor_msgs.Imu{
		Header: std_msgs.Header{
			Seq: 1,
		},
		OrientationCovariance:        [9]float64{0, 0, 0, 0, 0.2, 0, 0, 0, 0},
		AngularVelocityCovariance:    [9]float64{0, 0, 15, 0, 0, 0, 0, 0, 0},
		LinearAccelerationCovariance: [9]float64{0, 0, 0, 0, 0, 0, 0, 0, 13.7},
	}

	for _, pub := range []string{
		"cpp",
		"go",
		"rostopic",
	} {
		t.Run(pub, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			var expected interface{}
			switch pub {
			case "cpp":
				expected = expected1
				p := newContainer(t, "node-pub", m.IP())
				defer p.close()

			case "go":
				expected = expected1
				tp := newTestPublisher(t, m.IP(), func(header prototcp.HeaderSubscriber, conn *prototcp.Conn) {
					msgMd5, err := msgproc.MD5(TestMessage{})
					require.NoError(t, err)

					require.Equal(t, "/myns/goroslib", header.Callerid)
					require.Equal(t, "/myns/test_topic", header.Topic)
					require.Equal(t, "goroslib/TestMessage", header.Type)
					require.Equal(t, msgMd5, header.Md5sum)

					err = conn.WriteHeader(&prototcp.HeaderPublisher{
						Callerid: "myns/goroslib_pub",
						Md5sum:   msgMd5,
						Topic:    "/myns/test_topic",
						Type:     "/myns/TestMessage",
						Latching: 0,
					})
					require.NoError(t, err)

					err = conn.WriteMessage(expected1)
					require.NoError(t, err)
				})
				defer tp.close()

			case "rostopic":
				expected = expected2
				p := newContainer(t, "rostopic-pub", m.IP())
				defer p.close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			if expected == expected1 {
				recv := make(chan *TestMessage, 10)

				sub, err := NewSubscriber(SubscriberConf{
					Node:  n,
					Topic: "test_topic",
					Callback: func(msg *TestMessage) {
						recv <- msg
					},
				})
				require.NoError(t, err)
				defer sub.Close()

				require.Equal(t, expected, <-recv)
			} else {
				recv := make(chan *sensor_msgs.Imu, 10)

				sub, err := NewSubscriber(SubscriberConf{
					Node:  n,
					Topic: "test_topic",
					Callback: func(msg *sensor_msgs.Imu) {
						recv <- msg
					},
				})
				require.NoError(t, err)
				defer sub.Close()

				require.Equal(t, expected, <-recv)
			}
		})
	}
}

func TestSubscriberReadBeforePub(t *testing.T) {
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
		D: [2]uint32{222, 333},
	}

	for _, pub := range []string{
		"cpp",
		"go",
	} {
		t.Run(pub, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			recv := make(chan *TestMessage, 10)

			sub, err := NewSubscriber(SubscriberConf{
				Node:  n,
				Topic: "test_topic",
				Callback: func(msg *TestMessage) {
					recv <- msg
				},
			})
			require.NoError(t, err)
			defer sub.Close()

			switch pub {
			case "cpp":
				p := newContainer(t, "node-pub", m.IP())
				defer p.close()

			case "go":
				tp := newTestPublisher(t, m.IP(), func(header prototcp.HeaderSubscriber, conn *prototcp.Conn) {
					msgMd5, err := msgproc.MD5(TestMessage{})
					require.NoError(t, err)

					require.Equal(t, "/myns/goroslib", header.Callerid)
					require.Equal(t, "/myns/test_topic", header.Topic)
					require.Equal(t, "goroslib/TestMessage", header.Type)
					require.Equal(t, msgMd5, header.Md5sum)

					err = conn.WriteHeader(&prototcp.HeaderPublisher{
						Callerid: "myns/goroslib_pub",
						Md5sum:   msgMd5,
						Topic:    "/myns/test_topic",
						Type:     "/myns/TestMessage",
						Latching: 0,
					})
					require.NoError(t, err)

					err = conn.WriteMessage(&expected)
					require.NoError(t, err)
				})
				defer tp.close()
			}

			res := <-recv
			require.Equal(t, &expected, res)
		})
	}
}

func TestSubscriberReadUDP(t *testing.T) {
	expected := std_msgs.Int64MultiArray{}
	for i := int64(0); i < 400; i++ {
		expected.Data = append(expected.Data, i)
	}

	for _, pub := range []string{
		"cpp",
		"go",
	} {
		t.Run(pub, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			switch pub {
			case "cpp":
				p := newContainer(t, "node-pub-udp", m.IP())
				defer p.close()

			case "go":
				masterAddr, err := net.ResolveTCPAddr("tcp", m.IP()+":11311")
				require.NoError(t, err)

				host := findNodeHost(masterAddr)
				require.NotEqual(t, "", host)

				nodeAddr, err := net.ResolveTCPAddr("tcp", net.JoinHostPort(host, "0"))
				require.NoError(t, err)

				udprosServer, err := protoudp.NewServer(nodeAddr.IP.String() + ":9912")
				require.NoError(t, err)
				defer udprosServer.Close()

				apiSlaveServer, err := apislave.NewServer(nodeAddr.IP.String()+":9911", nodeAddr.IP, nodeAddr.Zone)
				require.NoError(t, err)
				defer apiSlaveServer.Close()

				go apiSlaveServer.Serve(func(req apislave.Request) apislave.Response {
					if req2, ok := req.(*apislave.RequestRequestTopic); ok {
						require.Equal(t, "/myns/goroslib", req2.CallerID)
						require.Equal(t, "/myns/test_topic", req2.Topic)

						protoHost := req2.Protocols[0][2].(string)
						protoPort := req2.Protocols[0][3].(int)

						msgMd5, err := msgproc.MD5(std_msgs.Int64MultiArray{})
						require.NoError(t, err)

						udpAddr, err := net.ResolveUDPAddr("udp",
							net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10)))
						require.NoError(t, err)

						go func() {
							time.Sleep(1 * time.Second)
							udprosServer.WriteMessage(1, 1, &expected, udpAddr)
						}()

						return apislave.ResponseRequestTopic{
							Code: 1,
							Protocol: []interface{}{
								"UDPROS",
								nodeAddr.IP.String(),
								udprosServer.Port(),
								1,
								1500,
								func() []byte {
									var buf bytes.Buffer
									protocommon.HeaderEncode(&buf, &protoudp.HeaderPublisher{
										Callerid: "/myns/goroslib_pub",
										Md5sum:   msgMd5,
										Topic:    "/myns/test_topic",
										Type:     "std_msgs/Int64MultiArray",
									})
									return buf.Bytes()[4:]
								}(),
							},
						}
					}
					return apislave.ErrorRes{}
				})

				apiMasterClient := apimaster.NewClient(masterAddr.String(), "myns/goroslib_pub")
				_, err = apiMasterClient.RegisterPublisher(
					"/myns/test_topic",
					"goroslib/TestMessage",
					apiSlaveServer.URL())
				require.NoError(t, err)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			recv := make(chan *std_msgs.Int64MultiArray, 10)

			sub, err := NewSubscriber(SubscriberConf{
				Node:  n,
				Topic: "test_topic",
				Callback: func(msg *std_msgs.Int64MultiArray) {
					recv <- msg
				},
				Protocol: UDP,
			})
			require.NoError(t, err)
			defer sub.Close()

			res := <-recv
			require.Equal(t, &expected, res)
		})
	}
}

func TestSubscriberQueue(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	sendDone := make(chan struct{})

	tp := newTestPublisher(t, m.IP(), func(header prototcp.HeaderSubscriber, conn *prototcp.Conn) {
		msgMd5, err := msgproc.MD5(std_msgs.Int64{})
		require.NoError(t, err)

		err = conn.WriteHeader(&prototcp.HeaderPublisher{
			Callerid: "myns/goroslib_pub",
			Md5sum:   msgMd5,
			Topic:    "/myns/test_topic",
			Type:     "/myns/TestMessage",
			Latching: 0,
		})
		require.NoError(t, err)

		conn.WriteMessage(&std_msgs.Int64{Data: 1})
		conn.WriteMessage(&std_msgs.Int64{Data: 2})
		conn.WriteMessage(&std_msgs.Int64{Data: 3})
		conn.WriteMessage(&std_msgs.Int64{Data: 4})
		conn.WriteMessage(&std_msgs.Int64{Data: 5})
		close(sendDone)
	})
	defer tp.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	recvCount := 0
	recvDone := make(chan struct{})

	sub, err := NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "test_topic",
		Callback: func(msg *std_msgs.Int64) {
			recvCount++
			<-sendDone

			if recvCount == 2 {
				close(recvDone)
			}
		},
		QueueSize: 1,
	})
	require.NoError(t, err)
	defer sub.Close()

	<-recvDone
}
