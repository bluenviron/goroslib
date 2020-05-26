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

      infty := make(chan int)
      <-infty
  }

*/
package goroslib

import (
	"fmt"
	"net"
	"os"
	"sync"
	"time"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/api-param"
	"github.com/aler9/goroslib/api-slave"
	"github.com/aler9/goroslib/tcpros"
)

func getOwnIp() string {
	ifaces, err := net.Interfaces()
	if err != nil {
		return ""
	}

	for _, i := range ifaces {
		if (i.Flags & net.FlagLoopback) != 0 {
			continue
		}
		addrs, err := i.Addrs()
		if err != nil {
			continue
		}
		for _, addr := range addrs {
			switch v := addr.(type) {
			case *net.IPNet:
				return v.IP.String()
			}
		}
	}
	return ""
}

type nodeEvent interface {
	isNodeEvent()
}

type nodeEventClose struct {
}

func (nodeEventClose) isNodeEvent() {}

type nodeEventTcprosClientNew struct {
	client *tcpros.Conn
}

func (nodeEventTcprosClientNew) isNodeEvent() {}

type nodeEventTcprosClientClose struct {
	client *tcpros.Conn
}

func (nodeEventTcprosClientClose) isNodeEvent() {}

type nodeEventTcprosClientSubscriber struct {
	client *tcpros.Conn
	header *tcpros.HeaderSubscriber
}

func (nodeEventTcprosClientSubscriber) isNodeEvent() {}

type nodeEventTcprosClientServiceClient struct {
	client *tcpros.Conn
	header *tcpros.HeaderServiceClient
}

func (nodeEventTcprosClientServiceClient) isNodeEvent() {}

type nodeEventPublisherUpdate struct {
	topic string
	urls  []string
}

func (nodeEventPublisherUpdate) isNodeEvent() {}

type nodeEventGetPublications struct {
	res chan [][]string
}

func (nodeEventGetPublications) isNodeEvent() {}

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
	// name of the node
	Name string

	// (optional) hostname or ip of this node, needed by other nodes
	// in order to communicate with it it.
	// if not provided, it will be set automatically
	Host string

	// (optional) port of the XML-RPC server of this node.
	// if not provided, it will be chosen by the OS
	XmlRpcPort uint16

	// (optional) port of the TCPROS server of this node.
	// if not provided, it will be chosen by the OS
	TcpRosPort uint16

	// hostname or ip of the master node
	MasterHost string

	// (optional) port of the HTTP API of the master node
	// if not provided, it will be set to 11311
	MasterPort uint16
}

// Node is a ROS Node, an entity that can create subscribers, publishers, service providers
// and service clients.
type Node struct {
	conf NodeConf

	events    chan nodeEvent
	terminate chan struct{}
	done      chan struct{}

	apiMasterClient  *api_master.Client
	apiParamClient   *api_param.Client
	apiSlaveServer   *api_slave.Server
	tcprosServer     *tcpros.Server
	tcprosClients    map[*tcpros.Conn]struct{}
	subscribers      map[string]*Subscriber
	publishers       map[string]*Publisher
	serviceProviders map[string]*ServiceProvider
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
	if conf.Host == "" {
		conf.Host = getOwnIp()
		if conf.Host == "" {
			return nil, fmt.Errorf("unable to set Host automatically")
		}
	}

	apiSlaveServer, err := api_slave.NewServer(conf.Host, conf.XmlRpcPort)
	if err != nil {
		return nil, err
	}

	tcprosServer, err := tcpros.NewServer(conf.Host, conf.TcpRosPort)
	if err != nil {
		apiSlaveServer.Close()
		return nil, err
	}

	apiMasterClient := api_master.NewClient(conf.MasterHost, conf.MasterPort, conf.Name)
	apiParamClient := api_param.NewClient(conf.MasterHost, conf.MasterPort, conf.Name)

	n := &Node{
		conf:             conf,
		events:           make(chan nodeEvent),
		terminate:        make(chan struct{}),
		done:             make(chan struct{}),
		apiMasterClient:  apiMasterClient,
		apiParamClient:   apiParamClient,
		apiSlaveServer:   apiSlaveServer,
		tcprosServer:     tcprosServer,
		tcprosClients:    make(map[*tcpros.Conn]struct{}),
		subscribers:      make(map[string]*Subscriber),
		publishers:       make(map[string]*Publisher),
		serviceProviders: make(map[string]*ServiceProvider),
	}

	go n.run()

	// request tcp_keepalive as in
	// http://docs.ros.org/melodic/api/roscpp/html/init_8cpp_source.html
	/*_, err = n.apiParamClient.HasParam("/tcp_keepalive")
	if err != nil {
		n.Close()
		return nil, err
	}*/

	/*type TempMsg struct {
		A uint8
		B []TestParent
	}*/

	// this is needed to check whether a node with the same name already exists
	/*n.rosoutPub, err = NewPublisher(PublisherConf{
		Node:  n,
		Topic: "/rosout",
		Msg:   &TempMsg{},
	})
	if err != nil {
		n.Close()
		return nil, err
	}*/

	// NewServiceProvider(n, "/[nodename]/get_loggers")

	// NewServiceProvider(n, "/[nodename]/set_logger_level")

	// HasParam("/use_sim_time")

	return n, nil
}

func (n *Node) run() {
	var serversWg sync.WaitGroup

	serversWg.Add(2)
	go n.runApiSlaveServer(&serversWg)
	go n.runTcprosServer(&serversWg)

	var clientsWg sync.WaitGroup

outer:
	for rawEvt := range n.events {
		switch evt := rawEvt.(type) {
		case nodeEventTcprosClientNew:
			n.tcprosClients[evt.client] = struct{}{}
			clientsWg.Add(1)
			go n.runTcprosClient(&clientsWg, evt.client)

		case nodeEventTcprosClientClose:
			delete(n.tcprosClients, evt.client)

		case nodeEventTcprosClientSubscriber:
			// pass client ownership to publisher, if exists
			delete(n.tcprosClients, evt.client)

			pub, ok := n.publishers[evt.header.Topic]
			if !ok {
				evt.client.Close()
				continue
			}

			pub.events <- publisherEventSubscriberNew{
				client: evt.client,
				header: evt.header,
			}

		case nodeEventTcprosClientServiceClient:
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

		case nodeEventSubscriberNew:
			_, ok := n.subscribers[evt.sub.conf.Topic]
			if ok {
				evt.err <- fmt.Errorf("Topic %s already subscribed", evt.sub.conf.Topic)
				continue
			}

			publisherUrls, err := n.apiMasterClient.RegisterSubscriber(api_master.RequestRegister{
				Topic:     evt.sub.conf.Topic[1:],
				TopicType: evt.sub.msgType,
				CallerUrl: n.apiSlaveServer.GetUrl(),
			})
			if err != nil {
				evt.err <- err
				continue
			}

			n.subscribers[evt.sub.conf.Topic] = evt.sub
			evt.err <- nil

			// send initial publishers list to subscriber
			evt.sub.events <- subscriberEventPublisherUpdate{publisherUrls}

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
				CallerUrl: n.apiSlaveServer.GetUrl(),
			})
			if err != nil {
				evt.err <- err
				continue
			}

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
				ServiceUrl: n.tcprosServer.GetUrl(),
				CallerUrl:  n.apiSlaveServer.GetUrl(),
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
				evt.res <- nil

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
	for {
		rawReq, err := n.apiSlaveServer.Read()
		if err != nil {
			break
		}

		switch req := rawReq.(type) {
		case *api_slave.RequestGetBusInfo:
			n.apiSlaveServer.Write(api_slave.ResponseGetBusInfo{
				Code:          1,
				StatusMessage: "bus info",
				// TODO: provide bus infos in this format:
				// connectionId, destinationId, direction (i, o, b), transport, topic,connected
				// {"1", "/rosout", "o", "tcpros", "/rosout", "1"}
				// [ 1, /rosout, o, tcpros, /rosout, 1, TCPROS connection on port 46477 to [127.0.0.1:51790 on socket 8] ]
				BusInfo: [][]string{},
			})

		case *api_slave.RequestGetPid:
			n.apiSlaveServer.Write(api_slave.ResponseGetPid{
				Code:          1,
				StatusMessage: "",
				Pid:           os.Getpid(),
			})

		case *api_slave.RequestGetPublications:
			publications := make(chan [][]string)
			n.events <- nodeEventGetPublications{publications}
			res := <-publications

			n.apiSlaveServer.Write(api_slave.ResponseGetPublications{
				Code:          1,
				StatusMessage: "",
				TopicList:     res,
			})

		case *api_slave.RequestPublisherUpdate:
			n.events <- nodeEventPublisherUpdate{
				topic: req.Topic,
				urls:  req.PublisherUrls,
			}

			n.apiSlaveServer.Write(api_slave.ResponsePublisherUpdate{
				Code:          1,
				StatusMessage: "",
			})

		case *api_slave.RequestRequestTopic:
			// Do not check here whether the topic exists or not,
			// just send the TCPROS port.
			// The check on the existence of the topic will take place in the
			// TCPROS connection.
			n.apiSlaveServer.Write(api_slave.ResponseRequestTopic{
				Code:          1,
				StatusMessage: "",
				Proto: api_slave.TopicProtocol{
					Name: "TCPROS",
					Host: n.conf.Host,
					Port: int(n.tcprosServer.GetPort()),
				},
			})

		case *api_slave.RequestShutdown:
			n.apiSlaveServer.Write(api_slave.ResponseShutdown{
				Code:          1,
				StatusMessage: "",
			})

			n.events <- nodeEventClose{}
		}
	}

	wg.Done()
}

func (n *Node) runTcprosServer(wg *sync.WaitGroup) {
	for {
		client, err := n.tcprosServer.Accept()
		if err != nil {
			break
		}

		n.events <- nodeEventTcprosClientNew{client}
	}

	wg.Done()
}

func (n *Node) runTcprosClient(wg *sync.WaitGroup, client *tcpros.Conn) {
	ok := func() bool {
		rawHeader, err := client.ReadHeaderRaw()
		if err != nil {
			return false
		}

		if _, ok := rawHeader["topic"]; ok {
			var header tcpros.HeaderSubscriber
			err = rawHeader.Decode(&header)
			if err != nil {
				return false
			}

			n.events <- nodeEventTcprosClientSubscriber{
				client: client,
				header: &header,
			}
			return true

		} else if _, ok := rawHeader["service"]; ok {
			var header tcpros.HeaderServiceClient
			err = rawHeader.Decode(&header)
			if err != nil {
				return false
			}

			n.events <- nodeEventTcprosClientServiceClient{
				client: client,
				header: &header,
			}
			return true
		}

		return false
	}()
	if !ok {
		client.Close()
		n.events <- nodeEventTcprosClientClose{client}
	}

	wg.Done()
}

// InfoNode contains informations about a node.
type InfoNode struct {
	PublishedTopics  map[string]struct{}
	SubscribedTopics map[string]struct{}
	ProvidedServices map[string]struct{}
	Hostname         string
	Port             uint16
}

// GetNodes returns all the nodes connected to the master.
func (n *Node) GetNodes() (map[string]*InfoNode, error) {
	sstate, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, err
	}

	ret := make(map[string]*InfoNode)

	initEntry := func(node string) {
		if _, ok := ret[node]; !ok {
			ret[node] = &InfoNode{
				PublishedTopics:  make(map[string]struct{}),
				SubscribedTopics: make(map[string]struct{}),
				ProvidedServices: make(map[string]struct{}),
			}
		}
	}

	for _, entry := range sstate.PublishedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].PublishedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range sstate.SubscribedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].SubscribedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range sstate.ProvidedServices {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].ProvidedServices[entry.Name] = struct{}{}
		}
	}

	for node, info := range ret {
		ur, err := n.apiMasterClient.LookupNode(api_master.RequestLookup{
			Name: node,
		})
		if err != nil {
			return nil, fmt.Errorf("lookupNode: %v", err)
		}

		hostname, port, err := parseUrl(ur)
		if err != nil {
			return nil, err
		}

		info.Hostname = hostname
		info.Port = port
	}

	return ret, nil
}

// GetMachines returns all the machines connected to the master through a node.
func (n *Node) GetMachines() (map[string]struct{}, error) {
	// this is like its equivalent in python
	// https://docs.ros.org/melodic/api/rosnode/html/rosnode-pysrc.html#get_machines_by_nodes

	nodes, err := n.GetNodes()
	if err != nil {
		return nil, err
	}

	ret := make(map[string]struct{})
	for _, info := range nodes {
		ret[info.Hostname] = struct{}{}
	}

	return ret, nil
}

// InfoTopic contains informations about a topic.
type InfoTopic struct {
	Type        string
	Publishers  map[string]struct{}
	Subscribers map[string]struct{}
}

// GetTopics returns all the topics published by nodes connected to the master.
func (n *Node) GetTopics() (map[string]*InfoTopic, error) {
	sstate, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %v", err)
	}

	ttypes, err := n.apiMasterClient.GetTopicTypes()
	if err != nil {
		return nil, fmt.Errorf("getTopicTypes: %v", err)
	}

	ret := make(map[string]*InfoTopic)

	for _, entry := range ttypes {
		ret[entry.Name] = &InfoTopic{
			Type:        entry.Type,
			Publishers:  make(map[string]struct{}),
			Subscribers: make(map[string]struct{}),
		}
	}

	for _, entry := range sstate.PublishedTopics {
		if _, ok := ret[entry.Name]; !ok {
			continue
		}
		for _, node := range entry.Nodes {
			ret[entry.Name].Publishers[node] = struct{}{}
		}
	}

	for _, entry := range sstate.SubscribedTopics {
		if _, ok := ret[entry.Name]; !ok {
			continue
		}
		for _, node := range entry.Nodes {
			ret[entry.Name].Subscribers[node] = struct{}{}
		}
	}

	return ret, nil
}

// InfoService contains informations about a service.
type InfoService struct {
	Providers map[string]struct{}
	Hostname  string
	Port      uint16
}

// GetServices returns all the services provided by nodes connected to the server.
func (n *Node) GetServices() (map[string]*InfoService, error) {
	sstate, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %v", err)
	}

	ret := make(map[string]*InfoService)

	for _, entry := range sstate.ProvidedServices {
		if _, ok := ret[entry.Name]; !ok {
			ret[entry.Name] = &InfoService{
				Providers: make(map[string]struct{}),
			}
		}

		for _, node := range entry.Nodes {
			ret[entry.Name].Providers[node] = struct{}{}
		}

		ur, err := n.apiMasterClient.LookupService(api_master.RequestLookup{
			Name: entry.Name,
		})
		if err != nil {
			return nil, fmt.Errorf("lookupService: %v", err)
		}

		hostname, port, err := parseUrl(ur)
		if err != nil {
			return nil, err
		}

		ret[entry.Name].Hostname = hostname
		ret[entry.Name].Port = port
	}

	return ret, nil
}

// PingNode send a ping request to a given node, wait for the reply and returns
// the elapsed time.
func (n *Node) PingNode(name string) (time.Duration, error) {
	ur, err := n.apiMasterClient.LookupNode(api_master.RequestLookup{
		Name: name,
	})
	if err != nil {
		return 0, err
	}

	hostname, port, err := parseUrl(ur)
	if err != nil {
		return 0, err
	}

	xcs := api_slave.NewClient(hostname, port, n.conf.Name)

	start := time.Now()

	_, err = xcs.GetPid()
	if err != nil {
		return 0, err
	}

	return time.Since(start), nil
}

// KillNode send a kill request to a given node.
func (n *Node) KillNode(name string) error {
	ur, err := n.apiMasterClient.LookupNode(api_master.RequestLookup{
		Name: name,
	})
	if err != nil {
		return err
	}

	hostname, port, err := parseUrl(ur)
	if err != nil {
		return err
	}

	xcs := api_slave.NewClient(hostname, port, n.conf.Name)

	err = xcs.Shutdown(api_slave.RequestShutdown{
		Reason: "",
	})
	if err != nil {
		return err
	}

	return nil
}

// GetParamBool returns a bool parameter from the master.
func (n *Node) GetParamBool(key string) (bool, error) {
	return n.apiParamClient.GetParamBool(api_param.RequestGetParam{
		Key: key,
	})
}

// GetParamInt returns an int parameter from the master.
func (n *Node) GetParamInt(key string) (int, error) {
	return n.apiParamClient.GetParamInt(api_param.RequestGetParam{
		Key: key,
	})
}

// GetParamString returns a string parameter from the master.
func (n *Node) GetParamString(key string) (string, error) {
	return n.apiParamClient.GetParamString(api_param.RequestGetParam{
		Key: key,
	})
}

// SetParamBool sets a bool parameter in the master.
func (n *Node) SetParamBool(key string, val bool) error {
	return n.apiParamClient.SetParamBool(api_param.RequestSetParamBool{
		Key: key,
		Val: val,
	})
}

// SetParamInt sets an int parameter in the master.
func (n *Node) SetParamInt(key string, val int) error {
	return n.apiParamClient.SetParamInt(api_param.RequestSetParamInt{
		Key: key,
		Val: val,
	})
}

// SetParamString sets a string parameter in the master.
func (n *Node) SetParamString(key string, val string) error {
	return n.apiParamClient.SetParamString(api_param.RequestSetParamString{
		Key: key,
		Val: val,
	})
}
