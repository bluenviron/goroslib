/*
Package goroslib is a library in pure Go that allows to build clients (nodes)
for the Robot Operating System (ROS).

Basic example (more are available at https://github.com/aler9/goroslib/tree/master/examples):

  package main

  import (
      "fmt"
      "github.com/aler9/goroslib"
      "github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
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
	"strconv"
	"sync"

	"github.com/aler9/goroslib/pkg/apimaster"
	"github.com/aler9/goroslib/pkg/apiparam"
	"github.com/aler9/goroslib/pkg/apislave"
	"github.com/aler9/goroslib/pkg/msgs/rosgraph_msgs"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
	"github.com/aler9/goroslib/pkg/xmlrpc"
)

type tcpClientSubscriberReq struct {
	client *prototcp.Conn
	header *prototcp.HeaderSubscriber
}

type tcpClientServiceClientReq struct {
	client *prototcp.Conn
	header *prototcp.HeaderServiceClient
}

type udpSubPublisherNewReq struct {
	sp        *subscriberPublisher
	chanFrame chan *protoudp.Frame
}

type udpSubPublisherCloseReq struct {
	sp   *subscriberPublisher
	done chan struct{}
}

type udpFrameReq struct {
	frame  *protoudp.Frame
	source *net.UDPAddr
}

type publisherUpdateReq struct {
	topic string
	urls  []string
}

type getPublicationsReq struct {
	res chan [][]string
}

type subscriberRequestTopicReq struct {
	req *apislave.RequestRequestTopic
	res chan apislave.ResponseRequestTopic
}

type subscriberNewReq struct {
	sub *Subscriber
	err chan error
}

type publisherNewReq struct {
	pub *Publisher
	err chan error
}

type serviceProviderNewReq struct {
	sp  *ServiceProvider
	err chan error
}

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
	apiMasterClient     *apimaster.Client
	apiParamClient      *apiparam.Client
	apiSlaveServer      *apislave.Server
	apiSlaveServerUrl   string
	tcprosServer        *prototcp.Server
	tcprosServerPort    int
	tcprosServerUrl     string
	udprosServer        *protoudp.Server
	udprosServerPort    int
	tcprosClients       map[*prototcp.Conn]struct{}
	udprosSubPublishers map[*subscriberPublisher]chan *protoudp.Frame
	subscribers         map[string]*Subscriber
	publishers          map[string]*Publisher
	serviceProviders    map[string]*ServiceProvider
	publisherLastId     int
	rosoutPublisher     *Publisher

	tcpClientNew           chan *prototcp.Conn
	tcpClientClose         chan *prototcp.Conn
	tcpClientSubscriber    chan tcpClientSubscriberReq
	tcpClientServiceClient chan tcpClientServiceClientReq
	udpSubPublisherNew     chan udpSubPublisherNewReq
	udpSubPublisherClose   chan udpSubPublisherCloseReq
	udpFrame               chan udpFrameReq
	publisherUpdate        chan publisherUpdateReq
	getPublications        chan getPublicationsReq
	subscriberRequestTopic chan subscriberRequestTopicReq
	subscriberNew          chan subscriberNewReq
	subscriberClose        chan *Subscriber
	publisherNew           chan publisherNewReq
	publisherClose         chan *Publisher
	serviceProviderNew     chan serviceProviderNewReq
	serviceProviderClose   chan *ServiceProvider
	shutdown               chan struct{}
	terminate              chan struct{}
	done                   chan struct{}
}

// NewNode allocates a Node. See NodeConf for the options.
func NewNode(conf NodeConf) (*Node, error) {
	if len(conf.Name) == 0 {
		return nil, fmt.Errorf("Name not provided")
	}
	if conf.Name[0] != '/' {
		return nil, fmt.Errorf("Name must begin with a slash (/)")
	}
	if len(conf.MasterHost) == 0 {
		return nil, fmt.Errorf("MasterHost not provided")
	}
	if conf.MasterPort == 0 {
		conf.MasterPort = 11311
	}

	// solve master address once
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
	if err != nil {
		return nil, err
	}

	apiMasterClient := apimaster.NewClient(masterIp.String(), conf.MasterPort, conf.Name)
	apiParamClient := apiparam.NewClient(masterIp.String(), conf.MasterPort, conf.Name)

	apiSlaveServer, err := apislave.NewServer(conf.ApislavePort)
	if err != nil {
		return nil, err
	}
	apiSlaveServerUrl := xmlrpc.ServerUrl(nodeIp.String(), apiSlaveServer.Port()) // get port in case it has been set automatically

	tcprosServer, err := prototcp.NewServer(conf.TcprosPort)
	if err != nil {
		apiSlaveServer.Close()
		return nil, err
	}
	tcprosServerPort := tcprosServer.Port() // get port in case it has been set automatically
	tcprosServerUrl := prototcp.ServerUrl(nodeIp.String(), tcprosServerPort)

	udprosServer, err := protoudp.NewServer(conf.UdprosPort)
	if err != nil {
		tcprosServer.Close()
		apiSlaveServer.Close()
		return nil, err
	}
	udprosServerPort := udprosServer.Port() // get port in case it has been set automatically

	n := &Node{
		conf:                   conf,
		masterIp:               masterIp,
		nodeIp:                 nodeIp,
		apiMasterClient:        apiMasterClient,
		apiParamClient:         apiParamClient,
		apiSlaveServer:         apiSlaveServer,
		apiSlaveServerUrl:      apiSlaveServerUrl,
		tcprosServer:           tcprosServer,
		tcprosServerPort:       tcprosServerPort,
		tcprosServerUrl:        tcprosServerUrl,
		udprosServer:           udprosServer,
		udprosServerPort:       udprosServerPort,
		tcprosClients:          make(map[*prototcp.Conn]struct{}),
		udprosSubPublishers:    make(map[*subscriberPublisher]chan *protoudp.Frame),
		subscribers:            make(map[string]*Subscriber),
		publishers:             make(map[string]*Publisher),
		serviceProviders:       make(map[string]*ServiceProvider),
		tcpClientNew:           make(chan *prototcp.Conn),
		tcpClientClose:         make(chan *prototcp.Conn),
		tcpClientSubscriber:    make(chan tcpClientSubscriberReq),
		tcpClientServiceClient: make(chan tcpClientServiceClientReq),
		udpSubPublisherNew:     make(chan udpSubPublisherNewReq),
		udpSubPublisherClose:   make(chan udpSubPublisherCloseReq),
		udpFrame:               make(chan udpFrameReq),
		publisherUpdate:        make(chan publisherUpdateReq),
		getPublications:        make(chan getPublicationsReq),
		subscriberRequestTopic: make(chan subscriberRequestTopicReq),
		subscriberNew:          make(chan subscriberNewReq),
		subscriberClose:        make(chan *Subscriber),
		publisherNew:           make(chan publisherNewReq),
		publisherClose:         make(chan *Publisher),
		serviceProviderNew:     make(chan serviceProviderNewReq),
		serviceProviderClose:   make(chan *ServiceProvider),
		shutdown:               make(chan struct{}),
		terminate:              make(chan struct{}),
		done:                   make(chan struct{}),
	}

	go n.run()

	n.rosoutPublisher, err = NewPublisher(PublisherConf{
		Node:  n,
		Topic: "/rosout",
		Msg:   &rosgraph_msgs.Log{},
	})
	if err != nil {
		n.Close()
		return nil, err
	}

	return n, nil
}

func (n *Node) run() {
	defer close(n.done)

	var serversWg sync.WaitGroup

	serversWg.Add(3)
	go n.runApiSlaveServer(&serversWg)
	go n.runTcprosServer(&serversWg)
	go n.runUdprosServer(&serversWg)

	var clientsWg sync.WaitGroup

outer:
	for {
		select {
		case client := <-n.tcpClientNew:
			n.tcprosClients[client] = struct{}{}
			clientsWg.Add(1)
			go n.runTcprosClient(&clientsWg, client)

		case client := <-n.tcpClientClose:
			delete(n.tcprosClients, client)

		case req := <-n.tcpClientSubscriber:
			// pass client ownership to publisher, if exists
			delete(n.tcprosClients, req.client)

			pub, ok := n.publishers[req.header.Topic]
			if !ok {
				req.client.Close()
				continue
			}

			pub.subscriberTcpNew <- publisherSubscriberTcpNewReq{
				client: req.client,
				header: req.header,
			}

		case req := <-n.tcpClientServiceClient:
			// pass client ownership to service provider, if exists
			delete(n.tcprosClients, req.client)

			sp, ok := n.serviceProviders[req.header.Service]
			if !ok {
				req.client.Close()
				continue
			}

			sp.clientNew <- serviceProviderClientNewReq{
				client: req.client,
				header: req.header,
			}

		case req := <-n.udpSubPublisherNew:
			n.udprosSubPublishers[req.sp] = req.chanFrame

		case req := <-n.udpSubPublisherClose:
			delete(n.udprosSubPublishers, req.sp)
			close(req.done)

		case req := <-n.udpFrame:
			for sp, chanFrame := range n.udprosSubPublishers {
				if req.frame.ConnectionId == sp.udprosId &&
					req.source.IP.Equal(sp.udprosIp) {
					chanFrame <- req.frame
					break
				}
			}

		case req := <-n.publisherUpdate:
			sub, ok := n.subscribers[req.topic]
			if !ok {
				continue
			}

			sub.publisherUpdate <- req.urls

		case req := <-n.getPublications:
			res := [][]string{}

			for _, pub := range n.publishers {
				res = append(res, []string{pub.conf.Topic, pub.msgType})
			}

			req.res <- res

		case req := <-n.subscriberRequestTopic:
			pub, ok := n.publishers[req.req.Topic]
			if !ok {
				req.res <- apislave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: "topic not found",
				}
				continue
			}

			pub.requestTopic <- publisherRequestTopicReq{req.req, req.res}

		case req := <-n.subscriberNew:
			_, ok := n.subscribers[req.sub.conf.Topic]
			if ok {
				req.err <- fmt.Errorf("Topic %s already subscribed", req.sub.conf.Topic)
				continue
			}

			res, err := n.apiMasterClient.RegisterSubscriber(
				req.sub.conf.Topic,
				req.sub.msgType,
				n.apiSlaveServerUrl)
			if err != nil {
				req.err <- err
				continue
			}

			n.subscribers[req.sub.conf.Topic] = req.sub
			req.err <- nil

			// send initial publishers list to subscriber
			req.sub.publisherUpdate <- res.Uris

		case sub := <-n.subscriberClose:
			delete(n.subscribers, sub.conf.Topic)
			close(sub.nodeTerminate)

		case req := <-n.publisherNew:
			_, ok := n.publishers[req.pub.conf.Topic]
			if ok {
				req.err <- fmt.Errorf("Topic %s already published", req.pub.conf.Topic)
				continue
			}

			_, err := n.apiMasterClient.RegisterPublisher(
				req.pub.conf.Topic,
				req.pub.msgType,
				n.apiSlaveServerUrl)
			if err != nil {
				req.err <- err
				continue
			}

			n.publisherLastId += 1
			req.pub.id = n.publisherLastId
			n.publishers[req.pub.conf.Topic] = req.pub
			req.err <- nil

		case pub := <-n.publisherClose:
			delete(n.publishers, pub.conf.Topic)
			close(pub.nodeTerminate)

		case req := <-n.serviceProviderNew:
			_, ok := n.serviceProviders[req.sp.conf.Service]
			if ok {
				req.err <- fmt.Errorf("Service %s already provided", req.sp.conf.Service)
				continue
			}

			err := n.apiMasterClient.RegisterService(
				req.sp.conf.Service,
				n.tcprosServerUrl,
				n.apiSlaveServerUrl)
			if err != nil {
				req.err <- err
				continue
			}

			n.serviceProviders[req.sp.conf.Service] = req.sp
			req.err <- nil

		case sp := <-n.serviceProviderClose:
			delete(n.serviceProviders, sp.conf.Service)
			close(sp.nodeTerminate)

		case <-n.shutdown:
			break outer
		}
	}

	// consume queue
	go func() {
		for {
			select {
			case <-n.tcpClientNew:

			case <-n.tcpClientClose:

			case <-n.tcpClientSubscriber:

			case <-n.tcpClientServiceClient:

			case <-n.udpSubPublisherNew:

			case req, ok := <-n.udpSubPublisherClose:
				if !ok {
					return
				}
				close(req.done)

			case <-n.udpFrame:

			case <-n.publisherUpdate:

			case req := <-n.getPublications:
				req.res <- [][]string{}

			case req := <-n.subscriberRequestTopic:
				req.res <- apislave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: "terminating",
				}

			case req := <-n.subscriberNew:
				req.err <- fmt.Errorf("terminated")

			case <-n.subscriberClose:

			case req := <-n.publisherNew:
				req.err <- fmt.Errorf("terminated")

			case <-n.publisherClose:

			case req := <-n.serviceProviderNew:
				req.err <- fmt.Errorf("terminated")

			case <-n.serviceProviderClose:

			case <-n.shutdown:
			}
		}
	}()

	// close all servers
	n.apiSlaveServer.Close()
	n.tcprosServer.Close()
	n.udprosServer.Close()
	serversWg.Wait()

	// close all clients
	for c := range n.tcprosClients {
		c.Close()
	}
	clientsWg.Wait()
	for _, sub := range n.subscribers {
		sub.shutdown <- struct{}{}
		close(sub.nodeTerminate)
	}
	for _, pub := range n.publishers {
		pub.shutdown <- struct{}{}
		close(pub.nodeTerminate)
	}
	for _, sp := range n.serviceProviders {
		sp.shutdown <- struct{}{}
		close(sp.nodeTerminate)
	}

	if n.rosoutPublisher != nil {
		n.rosoutPublisher.Close()
	}

	<-n.terminate

	close(n.tcpClientNew)
	close(n.tcpClientClose)
	close(n.tcpClientSubscriber)
	close(n.tcpClientServiceClient)
	close(n.udpSubPublisherNew)
	close(n.udpSubPublisherClose)
	close(n.udpFrame)
	close(n.publisherUpdate)
	close(n.getPublications)
	close(n.subscriberRequestTopic)
	close(n.subscriberNew)
	close(n.subscriberClose)
	close(n.publisherNew)
	close(n.publisherClose)
	close(n.serviceProviderNew)
	close(n.serviceProviderClose)
	close(n.shutdown)
}

// Close closes a Node and shuts down all its operations.
func (n *Node) Close() error {
	n.shutdown <- struct{}{}
	close(n.terminate)
	<-n.done
	return nil
}
