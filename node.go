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
}

type nodeEventClose struct {
}

type nodeEventTcprosClientNew struct {
	client *tcpros.Conn
}

type nodeEventTcprosClientClose struct {
	client *tcpros.Conn
}

type nodeEventTcprosClientSubscriber struct {
	client *tcpros.Conn
	header *tcpros.HeaderSubscriber
}

type nodeEventTcprosClientServiceClient struct {
	client *tcpros.Conn
	header *tcpros.HeaderServiceClient
}

type nodeEventPublisherUpdate struct {
	topic string
	urls  []string
}

type nodeEventSubscriberNew struct {
	sub     *Subscriber
	chanErr chan error
}

type nodeEventSubscriberClose struct {
	sub *Subscriber
}

type nodeEventPublisherNew struct {
	pub     *Publisher
	chanErr chan error
}

type nodeEventPublisherClose struct {
	pub *Publisher
}

type nodeEventServiceProviderNew struct {
	sp      *ServiceProvider
	chanErr chan error
}

type nodeEventServiceProviderClose struct {
	sp *ServiceProvider
}

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

	chanEvents chan nodeEvent
	chanDone   chan struct{}

	masterClient     *api_master.Client
	paramClient      *api_param.Client
	slaveServer      *api_slave.Server
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

	masterClient, err := api_master.NewClient(conf.MasterHost, conf.MasterPort, conf.Name)
	if err != nil {
		return nil, err
	}

	paramClient, err := api_param.NewClient(conf.MasterHost, conf.MasterPort, conf.Name)
	if err != nil {
		masterClient.Close()
		return nil, err
	}

	slaveServer, err := api_slave.NewServer(conf.Host, conf.XmlRpcPort)
	if err != nil {
		masterClient.Close()
		paramClient.Close()
		return nil, err
	}

	tcprosServer, err := tcpros.NewServer(conf.Host, conf.TcpRosPort)
	if err != nil {
		masterClient.Close()
		paramClient.Close()
		slaveServer.Close()
		return nil, err
	}

	n := &Node{
		conf:             conf,
		chanEvents:       make(chan nodeEvent),
		chanDone:         make(chan struct{}),
		masterClient:     masterClient,
		paramClient:      paramClient,
		slaveServer:      slaveServer,
		tcprosServer:     tcprosServer,
		tcprosClients:    make(map[*tcpros.Conn]struct{}),
		subscribers:      make(map[string]*Subscriber),
		publishers:       make(map[string]*Publisher),
		serviceProviders: make(map[string]*ServiceProvider),
	}

	go n.run()

	// request tcp_keepalive as in
	// http://docs.ros.org/melodic/api/roscpp/html/init_8cpp_source.html
	/*_, err = n.paramClient.HasParam("/tcp_keepalive")
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

// Close closes a Node and shuts down all its operations.
func (n *Node) Close() error {
	n.chanEvents <- nodeEventClose{}
	<-n.chanDone
	return nil
}

func (n *Node) run() {
	defer func() { n.chanDone <- struct{}{} }()

	var wg sync.WaitGroup

	wg.Add(1)
	go n.runApiSlaveServer(&wg)

	wg.Add(1)
	go n.runTcprosServer(&wg)

outer:
	for rawEvt := range n.chanEvents {
		switch evt := rawEvt.(type) {
		case nodeEventClose:
			break outer

		case nodeEventTcprosClientNew:
			n.tcprosClients[evt.client] = struct{}{}
			wg.Add(1)
			go n.runTcprosClient(&wg, evt.client)

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

			pub.chanEvents <- publisherEventSubscriberNew{
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

			sp.chanEvents <- serviceProviderEventClientNew{
				client: evt.client,
				header: evt.header,
			}

		case nodeEventPublisherUpdate:
			sub, ok := n.subscribers[evt.topic]
			if !ok {
				continue
			}
			sub.chanEvents <- subscriberEventPublisherUpdate{evt.urls}

		case nodeEventSubscriberNew:
			_, ok := n.subscribers[evt.sub.conf.Topic]
			if ok {
				evt.chanErr <- fmt.Errorf("Topic %s already subscribed", evt.sub.conf.Topic)
				continue
			}

			publisherUrls, err := n.masterClient.RegisterSubscriber(evt.sub.conf.Topic[1:],
				evt.sub.msgType, n.slaveServer.GetUrl())
			if err != nil {
				evt.chanErr <- err
				continue
			}

			n.subscribers[evt.sub.conf.Topic] = evt.sub
			evt.chanErr <- nil

			// send initial publishers list to subscriber
			evt.sub.chanEvents <- subscriberEventPublisherUpdate{publisherUrls}

		case nodeEventSubscriberClose:
			delete(n.subscribers, evt.sub.conf.Topic)

			n.masterClient.UnregisterSubscriber(evt.sub.conf.Topic[1:],
				n.slaveServer.GetUrl())

		case nodeEventPublisherNew:
			_, ok := n.publishers[evt.pub.conf.Topic]
			if ok {
				evt.chanErr <- fmt.Errorf("Topic %s already published", evt.pub.conf.Topic)
				continue
			}

			_, err := n.masterClient.RegisterPublisher(evt.pub.conf.Topic[1:],
				evt.pub.msgType, n.slaveServer.GetUrl())
			if err != nil {
				evt.chanErr <- err
				continue
			}

			n.publishers[evt.pub.conf.Topic] = evt.pub
			evt.chanErr <- nil

		case nodeEventPublisherClose:
			delete(n.publishers, evt.pub.conf.Topic)

			n.masterClient.UnregisterPublisher(evt.pub.conf.Topic[1:],
				n.slaveServer.GetUrl())

		case nodeEventServiceProviderNew:
			_, ok := n.serviceProviders[evt.sp.conf.Service]
			if ok {
				evt.chanErr <- fmt.Errorf("Service %s already provided", evt.sp.conf.Service)
				continue
			}

			err := n.masterClient.RegisterService(evt.sp.conf.Service[1:],
				n.tcprosServer.GetUrl(), n.slaveServer.GetUrl())
			if err != nil {
				evt.chanErr <- err
				continue
			}

			n.serviceProviders[evt.sp.conf.Service] = evt.sp
			evt.chanErr <- nil

		case nodeEventServiceProviderClose:
			delete(n.serviceProviders, evt.sp.conf.Service)

			n.masterClient.UnregisterService(evt.sp.conf.Service[1:],
				n.slaveServer.GetUrl())
		}
	}

	// consume queue
	go func() {
		for range n.chanEvents {
		}
	}()

	for _, sub := range n.subscribers {
		sub.Close()
	}

	for _, pub := range n.publishers {
		pub.Close()
	}

	for _, sp := range n.serviceProviders {
		sp.Close()
	}

	for c := range n.tcprosClients {
		c.Close()
	}

	n.paramClient.Close()
	n.masterClient.Close()
	n.slaveServer.Close()
	n.tcprosServer.Close()

	wg.Wait()

	close(n.chanEvents)
}

func (n *Node) runApiSlaveServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		rawReq, err := n.slaveServer.Read()
		if err != nil {
			return
		}

		switch req := rawReq.(type) {
		case *api_slave.ReqPublisherUpdate:
			n.chanEvents <- nodeEventPublisherUpdate{
				topic: req.Topic,
				urls:  req.PublisherUrls,
			}
			n.slaveServer.WritePublisherUpdate(1, "")

		case *api_slave.ReqRequestTopic:
			// Do not check here whether the topic exists or not,
			// just send the TCPROS port.
			// The check on the existence of the topic will take place in the
			// TCPROS connection.
			n.slaveServer.WriteRequestTopic(1, "", api_slave.TopicProtocol{
				Name: "TCPROS",
				Host: n.conf.Host,
				Port: int(n.tcprosServer.GetPort()),
			})

		case *api_slave.ReqShutdown:
			n.chanEvents <- nodeEventClose{}
		}
	}
}

func (n *Node) runTcprosServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		client, err := n.tcprosServer.Accept()
		if err != nil {
			return
		}

		n.chanEvents <- nodeEventTcprosClientNew{client}
	}
}

func (n *Node) runTcprosClient(wg *sync.WaitGroup, client *tcpros.Conn) {
	defer wg.Done()

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

			n.chanEvents <- nodeEventTcprosClientSubscriber{
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

			n.chanEvents <- nodeEventTcprosClientServiceClient{
				client: client,
				header: &header,
			}
			return true
		}

		return false
	}()
	if !ok {
		client.Close()
		n.chanEvents <- nodeEventTcprosClientClose{client}
	}
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
	sstate, err := n.masterClient.GetSystemState()
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
		ur, err := n.masterClient.LookupNode(node)
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
	sstate, err := n.masterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %v", err)
	}

	ttypes, err := n.masterClient.GetTopicTypes()
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
	sstate, err := n.masterClient.GetSystemState()
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

		ur, err := n.masterClient.LookupService(entry.Name)
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
	ur, err := n.masterClient.LookupNode(name)
	if err != nil {
		return 0, err
	}

	hostname, port, err := parseUrl(ur)
	if err != nil {
		return 0, err
	}

	xcs, err := api_slave.NewClient(hostname, port, n.conf.Name)
	if err != nil {
		return 0, err
	}
	defer xcs.Close()

	start := time.Now()

	_, err = xcs.GetPid()
	if err != nil {
		return 0, err
	}

	return time.Since(start), nil
}

// KillNode send a kill request to a given node.
func (n *Node) KillNode(name string) error {
	ur, err := n.masterClient.LookupNode(name)
	if err != nil {
		return err
	}

	hostname, port, err := parseUrl(ur)
	if err != nil {
		return err
	}

	xcs, err := api_slave.NewClient(hostname, port, n.conf.Name)
	if err != nil {
		return err
	}
	defer xcs.Close()

	err = xcs.Shutdown("")
	if err != nil {
		return err
	}

	return nil
}

// GetParamBool returns a bool parameter from the master.
func (n *Node) GetParamBool(name string) (bool, error) {
	return n.paramClient.GetParamBool(name)
}

// GetParamInt returns an int parameter from the master.
func (n *Node) GetParamInt(name string) (int, error) {
	return n.paramClient.GetParamInt(name)
}

// GetParamString returns a string parameter from the master.
func (n *Node) GetParamString(name string) (string, error) {
	return n.paramClient.GetParamString(name)
}

// SetParamBool sets a bool parameter in the master.
func (n *Node) SetParamBool(name string, val bool) error {
	return n.paramClient.SetParamBool(name, val)
}

// SetParamInt sets an int parameter in the master.
func (n *Node) SetParamInt(name string, val int) error {
	return n.paramClient.SetParamInt(name, val)
}

// SetParamString sets a string parameter in the master.
func (n *Node) SetParamString(name string, val string) error {
	return n.paramClient.SetParamString(name, val)
}
