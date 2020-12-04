package goroslib

import (
	"fmt"
	"reflect"
	"time"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/prototcp"
)

// ServiceClientConf is the configuration of a ServiceClient.
type ServiceClientConf struct {
	// node which the service client belongs to
	Node *Node

	// name of the service from which providers will be obtained
	Service string

	// an instance of the request that will be sent
	Req interface{}

	// an instance of the response that will be received
	Res interface{}

	// (optional) enable keep-alive packets, that are
	// useful when there's a firewall between nodes.
	EnableKeepAlive bool
}

// ServiceClient is a ROS service client, an entity that can send requests to
// service providers and receive responses.
type ServiceClient struct {
	conf ServiceClientConf
	conn *prototcp.Conn
}

// NewServiceClient allocates a ServiceClient. See ServiceClientConf for the options.
func NewServiceClient(conf ServiceClientConf) (*ServiceClient, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	rt := reflect.TypeOf(conf.Req)
	if rt.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Req must be a pointer")
	}
	if rt.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Req must be a pointer to a struct")
	}

	rt = reflect.TypeOf(conf.Res)
	if rt.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Res must be a pointer")
	}
	if rt.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Res must be a pointer to a struct")
	}

	return &ServiceClient{
		conf: conf,
	}, nil
}

// Close closes a ServiceClient and shuts down all its operations.
func (sc *ServiceClient) Close() error {
	if sc.conn != nil {
		sc.conn.Close()
	}
	return nil
}

// Call sends a request to a service provider and reads a response.
func (sc *ServiceClient) Call(req interface{}, res interface{}) error {
	if reflect.TypeOf(req) != reflect.TypeOf(sc.conf.Req) {
		panic("wrong req")
	}
	if reflect.TypeOf(res) != reflect.TypeOf(sc.conf.Res) {
		panic("wrong res")
	}

	connCreatedInThisCall := false
	if sc.conn == nil {
		err := sc.createConn()
		if err != nil {
			return err
		}
		connCreatedInThisCall = true
	}

	err := sc.conn.WriteMessage(req)
	if err != nil {
		sc.conn.Close()
		sc.conn = nil

		// if the connection was created previously, it could be damaged or
		// linked to an invalid provider.
		// do another try.
		if !connCreatedInThisCall {
			return sc.Call(req, res)
		}

		return err
	}

	err = sc.conn.ReadServiceResState()
	if err != nil {
		sc.conn.Close()
		sc.conn = nil

		// if the connection was created previously, it could be damaged or
		// linked to an invalid provider.
		// do another try.
		if !connCreatedInThisCall {
			return sc.Call(req, res)
		}

		return err
	}

	err = sc.conn.ReadMessage(res)
	if err != nil {
		sc.conn.Close()
		sc.conn = nil
		return err
	}

	return nil
}

func (sc *ServiceClient) createConn() error {
	res, err := sc.conf.Node.apiMasterClient.LookupService(
		sc.conf.Node.absoluteTopicName(sc.conf.Service))
	if err != nil {
		return fmt.Errorf("lookupService: %v", err)
	}

	srvMd5, err := msg.Md5Service(sc.conf.Req, sc.conf.Res)
	if err != nil {
		return err
	}

	address, err := urlToAddress(res.Uri)
	if err != nil {
		return err
	}

	conn, err := prototcp.NewClient(address)
	if err != nil {
		return err
	}

	if sc.conf.EnableKeepAlive {
		conn.NetConn().SetKeepAlive(true)
		conn.NetConn().SetKeepAlivePeriod(60 * time.Second)
	}

	err = conn.WriteHeader(&prototcp.HeaderServiceClient{
		Callerid:   sc.conf.Node.absoluteName(),
		Md5sum:     srvMd5,
		Persistent: 1,
		Service:    sc.conf.Node.absoluteTopicName(sc.conf.Service),
	})
	if err != nil {
		conn.Close()
		return err
	}

	var outHeader prototcp.HeaderServiceProvider
	err = conn.ReadHeader(&outHeader)
	if err != nil {
		conn.Close()
		return err
	}

	if outHeader.Md5sum != srvMd5 {
		conn.Close()
		return fmt.Errorf("wrong md5sum: expected %s, got %s",
			srvMd5, outHeader.Md5sum)
	}

	sc.conn = conn
	return nil
}
