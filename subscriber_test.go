package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
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

func TestSubscriberRegister(t *testing.T) {
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

	sub, err := NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "test_topic",
		Callback: func(msg *TestMessage) {
		},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	topics, err := n.GetTopics()
	require.NoError(t, err)

	topic, ok := topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/myns/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	sub.Close()
	time.Sleep(1 * time.Second)

	topics, err = n.GetTopics()
	require.NoError(t, err)

	topic, ok = topics["/myns/test_topic"]
	require.Equal(t, true, ok)

	_, ok = topic.Subscribers["/myns/goroslib"]
	require.Equal(t, false, ok)
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
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			var expected interface{}
			switch pub {
			case "cpp":
				expected = expected1
				p, err := newContainer("node-pub", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				expected = expected1
				p, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_pub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer p.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  p,
					Topic: "test_topic",
					Msg:   &TestMessage{},
				})
				require.NoError(t, err)
				defer pub.Close()

				pubTerminate := make(chan struct{})
				defer close(pubTerminate)

				go func() {
					t := time.NewTicker(1 * time.Second)
					defer t.Stop()

					for {
						select {
						case <-pubTerminate:
							return

						case <-t.C:
							pub.Write(expected)
						}
					}
				}()

			case "rostopic":
				expected = expected2
				p, err := newContainer("rostopic-pub", m.IP())
				require.NoError(t, err)
				defer p.close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib_sub",
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
				p, err := newContainer("node-pub", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				p, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_pub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer p.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  p,
					Topic: "test_topic",
					Msg:   &TestMessage{},
				})
				require.NoError(t, err)
				defer pub.Close()

				pubTerminate := make(chan struct{})
				defer close(pubTerminate)

				go func() {
					t := time.NewTicker(1 * time.Second)
					defer t.Stop()

					for {
						select {
						case <-pubTerminate:
							return

						case <-t.C:
							pub.Write(&expected)
						}
					}
				}()
			}

			res := <-recv
			require.Equal(t, &expected, res)
		})
	}
}

func TestSubscriberReadUdp(t *testing.T) {
	expected := std_msgs.Int64MultiArray{}
	for i := int64(0); i < 400; i++ {
		expected.Data = append(expected.Data, i)
	}

	for _, pub := range []string{
		"cpp",
		"go",
	} {
		t.Run(pub, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch pub {
			case "cpp":
				p, err := newContainer("node-pub-udp", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				p, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_pub",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer p.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  p,
					Topic: "test_topic",
					Msg:   &std_msgs.Int64MultiArray{},
				})
				require.NoError(t, err)
				defer pub.Close()

				pubTerminate := make(chan struct{})
				defer close(pubTerminate)

				go func() {
					t := time.NewTicker(1 * time.Second)
					defer t.Stop()

					for {
						select {
						case <-pubTerminate:
							return

						case <-t.C:
							pub.Write(&expected)
						}
					}
				}()
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
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib_pub",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer p.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  p,
		Topic: "test_topic",
		Msg:   &std_msgs.Int64{},
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

	recv := make(chan struct{})

	sub, err := NewSubscriber(SubscriberConf{
		Node:  n,
		Topic: "test_topic",
		Callback: func(msg *std_msgs.Int64) {
			recv <- struct{}{}
		},
		QueueSize: 1,
	})
	require.NoError(t, err)
	defer sub.Close()

	time.Sleep(1 * time.Second)

	pub.Write(&std_msgs.Int64{Data: 1})
	pub.Write(&std_msgs.Int64{Data: 2})
	pub.Write(&std_msgs.Int64{Data: 3})
	pub.Write(&std_msgs.Int64{Data: 4})
	pub.Write(&std_msgs.Int64{Data: 5})

	time.Sleep(1 * time.Second)

	<-recv
	<-recv

	select {
	case <-recv:
		t.Errorf("not expected")
	case <-time.After(1 * time.Second):
	}
}
