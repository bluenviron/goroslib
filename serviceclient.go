package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/apimaster"
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/prototcp"
)

// ServiceClientConf is the configuration of a ServiceClient.
type ServiceClientConf struct {
	// node which the service client belongs to
	Node *Node

	// name of the service from which providers will be obtained
	Service string

	// an instance of the request that will be sent
	Req interface{}

	// an instance of the reply that will be received
	Res interface{}
}

// ServiceClient is a ROS service client, an entity that can query service
// providers with requests and receive replies.
type ServiceClient struct {
	conf ServiceClientConf
	conn *prototcp.Conn
}

// NewServiceClient allocates a ServiceClient. See ServiceClientConf for the options.
func NewServiceClient(conf ServiceClientConf) (*ServiceClient, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if len(conf.Service) < 1 || conf.Service[0] != '/' {
		return nil, fmt.Errorf("Service must begin with /")
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

	res, err := conf.Node.apiMasterClient.LookupService(apimaster.RequestLookup{
		Name: conf.Service,
	})
	if err != nil {
		return nil, fmt.Errorf("lookupService: %v", err)
	}

	srvMd5, err := msg.Md5Service(conf.Req, conf.Res)
	if err != nil {
		return nil, err
	}

	host, port, err := parseUrl(res.Uri)
	if err != nil {
		return nil, err
	}

	conn, err := prototcp.NewClient(host, port)
	if err != nil {
		return nil, err
	}

	err = conn.WriteHeader(&prototcp.HeaderServiceClient{
		Callerid:   conf.Node.conf.Name,
		Md5sum:     srvMd5,
		Persistent: 1,
		Service:    conf.Service,
	})
	if err != nil {
		conn.Close()
		return nil, err
	}

	var outHeader prototcp.HeaderServiceProvider
	err = conn.ReadHeader(&outHeader)
	if err != nil {
		conn.Close()
		return nil, err
	}

	if outHeader.Md5sum != srvMd5 {
		conn.Close()
		return nil, fmt.Errorf("wrong md5sum: expected %s, got %s",
			srvMd5, outHeader.Md5sum)
	}

	return &ServiceClient{
		conf: conf,
		conn: conn,
	}, nil
}

// Close closes a ServiceClient and shuts down all its operations.
func (sc *ServiceClient) Close() error {
	sc.conn.Close()
	return nil
}

// Call performs the request to the service provider.
func (sc *ServiceClient) Call(req interface{}, res interface{}) error {
	if reflect.TypeOf(req) != reflect.TypeOf(sc.conf.Req) {
		panic("wrong req")
	}
	if reflect.TypeOf(res) != reflect.TypeOf(sc.conf.Res) {
		panic("wrong res")
	}

	err := sc.conn.WriteMessage(req)
	if err != nil {
		return err
	}

	err = sc.conn.ReadServiceResState()
	if err != nil {
		return err
	}

	err = sc.conn.ReadMessage(res)
	if err != nil {
		return err
	}

	return nil
}
