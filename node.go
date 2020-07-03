/*
Package goroslib is a library in pure Go that allows to build clients (nodes)
for the Robot Operating System (ROS).

Basic example (more are available at https://github.com/aler9/goroslib/tree/master/examples):

  package main

  import (
      "fmt"
      "github.com/aler9/goroslib"
      "github.com/aler9/goroslib/msgs/sensor_msgs"
  )

  func onMessage(msg *sensor_msgs.Imu) {
      fmt.Printf("Incoming: %+v\n", msg)
  }

  func main() {
      n, err := goroslib.NewNode(goroslib.NodeConf{
          Name:       "/goroslib",
          MasterHost: "127.0.0.1",
      })
      if err != nil {
          panic(err)
      }
      defer n.Close()

      sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
          Node:     n,
          Topic:    "/test_pub",
          Callback: onMessage,
      })
      if err != nil {
          panic(err)
      }
      defer sub.Close()

      select {}
  }

*/
package goroslib

import (
	"fmt"
	"net"
	"os"
	"strconv"
	"sync"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/api-param"
	"github.com/aler9/goroslib/api-slave"
	"github.com/aler9/goroslib/msgs/rosgraph_msgs"
	"github.com/aler9/goroslib/proto-common"
	"github.com/aler9/goroslib/proto-tcp"
	"github.com/aler9/goroslib/proto-udp"
	"github.com/aler9/goroslib/xmlrpc"
)

type nodeEvent interface {
	isNodeEvent()
}

type nodeEventClose struct{}

func (nodeEventClose) isNodeEvent() {}

type nodeEventTcpClientNew struct {
	client *proto_tcp.Conn
}

func (nodeEventTcpClientNew) isNodeEvent() {}

type nodeEventTcpClientClose struct {
	client *proto_tcp.Conn
}

func (nodeEventTcpClientClose) isNodeEvent() {}

type nodeEventTcpClientSubscriber struct {
	client *proto_tcp.Conn
	header *proto_tcp.HeaderSubscriber
}

func (nodeEventTcpClientSubscriber) isNodeEvent() {}

type nodeEventTcpClientServiceClient struct {
	client *proto_tcp.Conn
	header *proto_tcp.HeaderServiceClient
}

func (nodeEventTcpClientServiceClient) isNodeEvent() {}

type nodeEventUdpSubPublisherNew struct {
	sp        *subscriberPublisher
	chanFrame chan *proto_udp.Frame
}

func (nodeEventUdpSubPublisherNew) isNodeEvent() {}

type nodeEventUdpSubPublisherClose struct {
	sp   *subscriberPublisher
	done chan struct{}
}

func (nodeEventUdpSubPublisherClose) isNodeEvent() {}

type nodeEventUdpFrame struct {
	frame  *proto_udp.Frame
	source *net.UDPAddr
}

func (nodeEventUdpFrame) isNodeEvent() {}

type nodeEventPublisherUpdate struct {
	topic string
	urls  []string
}

func (nodeEventPublisherUpdate) isNodeEvent() {}

type nodeEventGetPublications struct {
	res chan [][]string
}

func (nodeEventGetPublications) isNodeEvent() {}

type nodeEventSubscriberRequestTopic struct {
	req *api_slave.RequestRequestTopic
	res chan api_slave.ResponseRequestTopic
}

func (nodeEventSubscriberRequestTopic) isNodeEvent() {}

type nodeEventSubscriberNew struct {
	sub *Subscriber
	err chan error
}

func (nodeEventSubscriberNew) isNodeEvent() {}

type nodeEventSubscriberClose struct {
	sub *Subscriber
}

func (nodeEventSubscriberClose) isNodeEvent() {}

type nodeEventPublisherNew struct {
	pub *Publisher
	err chan error
}

func (nodeEventPublisherNew) isNodeEvent() {}

type nodeEventPublisherClose struct {
	pub *Publisher
}

func (nodeEventPublisherClose) isNodeEvent() {}

type nodeEventServiceProviderNew struct {
	sp  *ServiceProvider
	err chan error
}

func (nodeEventServiceProviderNew) isNodeEvent() {}

type nodeEventServiceProviderClose struct {
	sp *ServiceProvider
}

func (nodeEventServiceProviderClose) isNodeEvent() {}

// NodeConf is the configuration of a Node.
type NodeConf struct {
	// hostname or ip of the master node
	MasterHost string

	// (optional) port of the HTTP API of the master node
	// if not provided, it will be set to 11311
	MasterPort int

	// name of this node
	Name string

	// (optional) hostname or ip of this node, needed by other nodes
	// in order to communicate with it it.
	// if not provided, it will be set automatically
	Host string

	// (optional) port of the Slave API server of this node.
	// if not provided, it will be chosen by the OS
	ApislavePort int

	// (optional) port of the TCPROS server of this node.
	// if not provided, it will be chosen by the OS
	TcprosPort int

	// (optional) port of the UDPROS server of this node.
	// if not provided, it will be chosen by the OS
	UdprosPort int
}

// Node is a ROS Node, an entity that can create subscribers, publishers, service providers
// and service clients.
type Node struct {
	conf                NodeConf
	masterIp            net.IP
	nodeIp              net.IP
	apiMasterClient     *api_master.Client
	apiParamClient      *api_param.Client
	apiSlaveServer      *api_slave.Server
	apiSlaveServerUrl   string
	tcprosServer        *proto_tcp.Server
	tcprosServerPort    int
	tcprosServerUrl     string
	udprosServer        *proto_udp.Server
	udprosServerPort    int
	tcprosClients       map[*proto_tcp.Conn]struct{}
	udprosSubPublishers map[*subscriberPublisher]chan *proto_udp.Frame
	subscribers         map[string]*Subscriber
	publishers          map[string]*Publisher
	serviceProviders    map[string]*ServiceProvider
	publisherLastId     int
	rosoutPublisher     *Publisher

	events    chan nodeEvent
	terminate chan struct{}
	done      chan struct{}
}

// NewNode allocates a Node. See NodeConf for the options.
func NewNode(conf NodeConf) (*Node, error) {
	if len(conf.Name) == 0 {
		return nil, fmt.Errorf("Name not provided")
	}
	if conf.Name[0] != '/' {
		return nil, fmt.Errorf("Name must begin with /")
	}
	if len(conf.MasterHost) == 0 {
		return nil, fmt.Errorf("MasterHost not provided")
	}
	if conf.MasterPort == 0 {
		conf.MasterPort = 11311
	}

	// solve master ip once
	masterIp, err := func() (net.IP, error) {
		addr, err := net.ResolveTCPAddr("tcp4", conf.MasterHost+":"+strconv.FormatInt(int64(conf.MasterPort), 10))
		if err != nil {
			return nil, fmt.Errorf("unable to solve master host: %s", err)
		}
		return addr.IP, nil
	}()
	if err != nil {
		return nil, err
	}

	// find an ip in the same subnet of the master
	if conf.Host == "" {
		conf.Host = func() string {
			ifaces, err := net.Interfaces()
			if err != nil {
				return ""
			}

			for _, i := range ifaces {
				addrs, err := i.Addrs()
				if err != nil {
					continue
				}

				for _, addr := range addrs {
					if v, ok := addr.(*net.IPNet); ok {
						if v.Contains(masterIp) {
							return v.IP.String()
						}
					}
				}
			}
			return ""
		}()
		if conf.Host == "" {
			return nil, fmt.Errorf("unable to set Host automatically")
		}
	}

	// solve node ip once
	nodeIp, err := func() (net.IP, error) {
		addr, err := net.ResolveTCPAddr("tcp4", conf.Host+":0")
		if err != nil {
			return nil, fmt.Errorf("unable to solve node host: %s", err)
		}
		return addr.IP, nil
	}()

	apiMasterClient := api_master.NewClient(masterIp.String(), conf.MasterPort, conf.Name)
	apiParamClient := api_param.NewClient(masterIp.String(), conf.MasterPort, conf.Name)

	apiSlaveServer, err := api_slave.NewServer(conf.ApislavePort)
	if err != nil {
		return nil, err
	}
	apiSlaveServerUrl := xmlrpc.ServerUrl(nodeIp.String(), apiSlaveServer.Port()) // get port in case it has been set automatically

	tcprosServer, err := proto_tcp.NewServer(conf.TcprosPort)
	if err != nil {
		apiSlaveServer.Close()
		return nil, err
	}
	tcprosServerPort := tcprosServer.Port() // get port in case it has been set automatically
	tcprosServerUrl := proto_tcp.ServerUrl(nodeIp.String(), tcprosServerPort)

	udprosServer, err := proto_udp.NewServer(conf.UdprosPort)
	if err != nil {
		tcprosServer.Close()
		apiSlaveServer.Close()
		return nil, err
	}
	udprosServerPort := udprosServer.Port() // get port in case it has been set automatically

	n := &Node{
		conf:                conf,
		masterIp:            masterIp,
		nodeIp:              nodeIp,
		apiMasterClient:     apiMasterClient,
		apiParamClient:      apiParamClient,
		apiSlaveServer:      apiSlaveServer,
		apiSlaveServerUrl:   apiSlaveServerUrl,
		tcprosServer:        tcprosServer,
		tcprosServerPort:    tcprosServerPort,
		tcprosServerUrl:     tcprosServerUrl,
		udprosServer:        udprosServer,
		udprosServerPort:    udprosServerPort,
		tcprosClients:       make(map[*proto_tcp.Conn]struct{}),
		udprosSubPublishers: make(map[*subscriberPublisher]chan *proto_udp.Frame),
		subscribers:         make(map[string]*Subscriber),
		publishers:          make(map[string]*Publisher),
		serviceProviders:    make(map[string]*ServiceProvider),
		events:              make(chan nodeEvent),
		terminate:           make(chan struct{}),
		done:                make(chan struct{}),
	}

	go n.run()

	rosoutPublisher, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "/rosout",
		Msg:   &rosgraph_msgs.Log{},
	})
	if err != nil {
		n.Close()
		return nil, err
	}
	n.rosoutPublisher = rosoutPublisher

	return n, nil
}

func (n *Node) run() {
	var serversWg sync.WaitGroup

	serversWg.Add(3)
	go n.runApiSlaveServer(&serversWg)
	go n.runTcprosServer(&serversWg)
	go n.runUdprosServer(&serversWg)

	var clientsWg sync.WaitGroup

outer:
	for rawEvt := range n.events {
		switch evt := rawEvt.(type) {
		case nodeEventTcpClientNew:
			n.tcprosClients[evt.client] = struct{}{}
			clientsWg.Add(1)
			go n.runTcprosClient(&clientsWg, evt.client)

		case nodeEventTcpClientClose:
			delete(n.tcprosClients, evt.client)

		case nodeEventTcpClientSubscriber:
			// pass client ownership to publisher, if exists
			delete(n.tcprosClients, evt.client)

			pub, ok := n.publishers[evt.header.Topic]
			if !ok {
				evt.client.Close()
				continue
			}

			pub.events <- publisherEventSubscriberTcpNew{
				client: evt.client,
				header: evt.header,
			}

		case nodeEventTcpClientServiceClient:
			// pass client ownership to service provider, if exists
			delete(n.tcprosClients, evt.client)

			sp, ok := n.serviceProviders[evt.header.Service]
			if !ok {
				evt.client.Close()
				continue
			}

			sp.events <- serviceProviderEventClientNew{
				client: evt.client,
				header: evt.header,
			}

		case nodeEventUdpSubPublisherNew:
			n.udprosSubPublishers[evt.sp] = evt.chanFrame

		case nodeEventUdpSubPublisherClose:
			delete(n.udprosSubPublishers, evt.sp)
			close(evt.done)

		case nodeEventUdpFrame:
			for sp, chanFrame := range n.udprosSubPublishers {
				if evt.frame.ConnectionId == sp.udprosId &&
					evt.source.IP.Equal(sp.udprosIp) {
					chanFrame <- evt.frame
					break
				}
			}

		case nodeEventPublisherUpdate:
			sub, ok := n.subscribers[evt.topic]
			if !ok {
				continue
			}

			sub.events <- subscriberEventPublisherUpdate{evt.urls}

		case nodeEventGetPublications:
			res := [][]string{}

			for _, pub := range n.publishers {
				res = append(res, []string{pub.conf.Topic, pub.msgType})
			}

			evt.res <- res

		case nodeEventSubscriberRequestTopic:
			pub, ok := n.publishers[evt.req.Topic]
			if !ok {
				evt.res <- api_slave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: "topic not found",
				}
				continue
			}

			pub.events <- publisherEventRequestTopic{evt.req, evt.res}

		case nodeEventSubscriberNew:
			_, ok := n.subscribers[evt.sub.conf.Topic]
			if ok {
				evt.err <- fmt.Errorf("Topic %s already subscribed", evt.sub.conf.Topic)
				continue
			}

			res, err := n.apiMasterClient.RegisterSubscriber(api_master.RequestRegister{
				Topic:     evt.sub.conf.Topic[1:],
				TopicType: evt.sub.msgType,
				CallerUrl: n.apiSlaveServerUrl,
			})
			if err != nil {
				evt.err <- err
				continue
			}

			n.subscribers[evt.sub.conf.Topic] = evt.sub
			evt.err <- nil

			// send initial publishers list to subscriber
			evt.sub.events <- subscriberEventPublisherUpdate{res.Uris}

		case nodeEventSubscriberClose:
			delete(n.subscribers, evt.sub.conf.Topic)
			close(evt.sub.nodeDone)

		case nodeEventPublisherNew:
			_, ok := n.publishers[evt.pub.conf.Topic]
			if ok {
				evt.err <- fmt.Errorf("Topic %s already published", evt.pub.conf.Topic)
				continue
			}

			_, err := n.apiMasterClient.RegisterPublisher(api_master.RequestRegister{
				Topic:     evt.pub.conf.Topic[1:],
				TopicType: evt.pub.msgType,
				CallerUrl: n.apiSlaveServerUrl,
			})
			if err != nil {
				evt.err <- err
				continue
			}

			n.publisherLastId += 1
			evt.pub.id = n.publisherLastId
			n.publishers[evt.pub.conf.Topic] = evt.pub
			evt.err <- nil

		case nodeEventPublisherClose:
			delete(n.publishers, evt.pub.conf.Topic)
			close(evt.pub.nodeDone)

		case nodeEventServiceProviderNew:
			_, ok := n.serviceProviders[evt.sp.conf.Service]
			if ok {
				evt.err <- fmt.Errorf("Service %s already provided", evt.sp.conf.Service)
				continue
			}

			err := n.apiMasterClient.RegisterService(api_master.RequestRegisterService{
				Service:    evt.sp.conf.Service[1:],
				ServiceUrl: n.tcprosServerUrl,
				CallerUrl:  n.apiSlaveServerUrl,
			})
			if err != nil {
				evt.err <- err
				continue
			}

			n.serviceProviders[evt.sp.conf.Service] = evt.sp
			evt.err <- nil

		case nodeEventServiceProviderClose:
			delete(n.serviceProviders, evt.sp.conf.Service)
			close(evt.sp.nodeDone)

		case nodeEventClose:
			break outer
		}
	}

	// use clientsWg for all remaining subscribers, publishers, service providers
	clientsWg.Add(len(n.subscribers) + len(n.publishers) + len(n.serviceProviders))

	// consume queue
	go func() {
		for rawEvt := range n.events {
			switch evt := rawEvt.(type) {
			case nodeEventGetPublications:
				evt.res <- [][]string{}

			case nodeEventUdpSubPublisherClose:
				close(evt.done)

			case nodeEventSubscriberRequestTopic:
				evt.res <- api_slave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: "terminating",
				}

			case nodeEventSubscriberNew:
				evt.err <- fmt.Errorf("terminated")

			case nodeEventSubscriberClose:
				clientsWg.Done()

			case nodeEventPublisherNew:
				evt.err <- fmt.Errorf("terminated")

			case nodeEventPublisherClose:
				clientsWg.Done()

			case nodeEventServiceProviderNew:
				evt.err <- fmt.Errorf("terminated")

			case nodeEventServiceProviderClose:
				clientsWg.Done()
			}
		}
	}()

	// close all servers and wait
	n.apiSlaveServer.Close()
	n.tcprosServer.Close()
	n.udprosServer.Close()
	serversWg.Wait()

	// close all clients and wait
	for c := range n.tcprosClients {
		c.Close()
	}
	for _, sub := range n.subscribers {
		sub.events <- subscriberEventClose{}
		close(sub.nodeDone)
	}
	for _, pub := range n.publishers {
		pub.events <- publisherEventClose{}
		close(pub.nodeDone)
	}
	for _, sp := range n.serviceProviders {
		sp.events <- serviceProviderEventClose{}
		close(sp.nodeDone)
	}
	clientsWg.Wait()

	if n.rosoutPublisher != nil {
		n.rosoutPublisher.Close()
	}

	// wait Close()
	<-n.terminate

	close(n.events)

	close(n.done)
}

// Close closes a Node and shuts down all its operations.
func (n *Node) Close() error {
	n.events <- nodeEventClose{}
	close(n.terminate)
	<-n.done
	return nil
}

func (n *Node) runApiSlaveServer(wg *sync.WaitGroup) {
	n.apiSlaveServer.Handle(func(rawReq api_slave.Request) api_slave.Response {
		switch req := rawReq.(type) {
		case *api_slave.RequestGetBusInfo:
			return api_slave.ResponseGetBusInfo{
				Code:          1,
				StatusMessage: "bus info",
				// TODO: provide bus infos in this format:
				// connectionId, destinationId, direction (i, o, b), transport, topic,connected
				// {"1", "/rosout", "o", "tcpros", "/rosout", "1"}
				// [ 1, /rosout, o, tcpros, /rosout, 1, TCPROS connection on port 46477 to [127.0.0.1:51790 on socket 8] ]
				BusInfo: [][]string{},
			}

		case *api_slave.RequestGetPid:
			return api_slave.ResponseGetPid{
				Code:          1,
				StatusMessage: "",
				Pid:           os.Getpid(),
			}

		case *api_slave.RequestGetPublications:
			resChan := make(chan [][]string)
			n.events <- nodeEventGetPublications{resChan}
			res := <-resChan

			return api_slave.ResponseGetPublications{
				Code:          1,
				StatusMessage: "",
				TopicList:     res,
			}

		case *api_slave.RequestPublisherUpdate:
			n.events <- nodeEventPublisherUpdate{
				topic: req.Topic,
				urls:  req.PublisherUrls,
			}

			return api_slave.ResponsePublisherUpdate{
				Code:          1,
				StatusMessage: "",
			}

		case *api_slave.RequestRequestTopic:
			resChan := make(chan api_slave.ResponseRequestTopic)
			n.events <- nodeEventSubscriberRequestTopic{req, resChan}
			res := <-resChan

			return res

		case *api_slave.RequestShutdown:
			n.events <- nodeEventClose{}

			return api_slave.ResponseShutdown{
				Code:          1,
				StatusMessage: "",
			}
		}

		return api_slave.ErrorRes{}
	})

	wg.Done()
}

func (n *Node) runTcprosServer(wg *sync.WaitGroup) {
	for {
		client, err := n.tcprosServer.Accept()
		if err != nil {
			break
		}

		n.events <- nodeEventTcpClientNew{client}
	}

	wg.Done()
}

func (n *Node) runUdprosServer(wg *sync.WaitGroup) {
	for {
		frame, source, err := n.udprosServer.ReadFrame()
		if err != nil {
			break
		}

		n.events <- nodeEventUdpFrame{frame, source}
	}

	wg.Done()
}

func (n *Node) runTcprosClient(wg *sync.WaitGroup, client *proto_tcp.Conn) {
	ok := func() bool {
		rawHeader, err := client.ReadHeaderRaw()
		if err != nil {
			return false
		}

		if _, ok := rawHeader["topic"]; ok {
			var header proto_tcp.HeaderSubscriber
			err = proto_common.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			n.events <- nodeEventTcpClientSubscriber{
				client: client,
				header: &header,
			}
			return true

		} else if _, ok := rawHeader["service"]; ok {
			var header proto_tcp.HeaderServiceClient
			err = proto_common.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			n.events <- nodeEventTcpClientServiceClient{
				client: client,
				header: &header,
			}
			return true
		}

		return false
	}()
	if !ok {
		client.Close()
		n.events <- nodeEventTcpClientClose{client}
	}

	wg.Done()
}
