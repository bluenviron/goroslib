package goroslib

import (
	"context"
	"fmt"
	"net"
	"reflect"
	"time"

	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/serviceproc"
)

// ServiceClientConf is the configuration of a ServiceClient.
type ServiceClientConf struct {
	// parent node.
	Node *Node

	// name of the service from which providers will be obtained.
	Name string

	// an instance of the service type.
	Srv interface{}

	// (optional) enable keep-alive packets, that are
	// useful when there's a firewall between nodes.
	EnableKeepAlive bool
}

// ServiceClient is a ROS service client, an entity that can send requests to
// service providers and receive responses.
type ServiceClient struct {
	conf   ServiceClientConf
	srvReq interface{}
	srvRes interface{}
	srvMD5 string
	conn   *prototcp.Conn
}

// NewServiceClient allocates a ServiceClient. See ServiceClientConf for the options.
func NewServiceClient(conf ServiceClientConf) (*ServiceClient, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Name == "" {
		return nil, fmt.Errorf("Name is empty")
	}

	if conf.Srv == nil {
		return nil, fmt.Errorf("Srv is empty")
	}

	if reflect.TypeOf(conf.Srv).Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Srv is not a pointer")
	}

	srvElem := reflect.ValueOf(conf.Srv).Elem().Interface()

	srvReq, srvRes, err := serviceproc.RequestResponse(srvElem)
	if err != nil {
		return nil, err
	}

	srvMD5, err := serviceproc.MD5(srvElem)
	if err != nil {
		return nil, err
	}

	conf.Name = conf.Node.applyCliRemapping(conf.Name)

	sc := &ServiceClient{
		conf:   conf,
		srvReq: srvReq,
		srvRes: srvRes,
		srvMD5: srvMD5,
	}

	sc.conf.Node.Log(LogLevelDebug, "service client '%s' created",
		sc.conf.Node.absoluteTopicName(conf.Name))

	return sc, nil
}

// Close closes a ServiceClient and shuts down all its operations.
func (sc *ServiceClient) Close() error {
	if sc.conn != nil {
		sc.conn.Close()
	}
	sc.conf.Node.Log(LogLevelDebug, "service client '%s' destroyed",
		sc.conf.Node.absoluteTopicName(sc.conf.Name))
	return nil
}

// Call sends a request to a service provider and reads a response.
func (sc *ServiceClient) Call(req interface{}, res interface{}) error {
	return sc.CallContext(context.Background(), req, res)
}

// CallContext sends a request to a service provider and reads a response.
// It allows to set a context that can be used to terminate the function.
func (sc *ServiceClient) CallContext(ctx context.Context, req interface{}, res interface{}) error {
	if reflect.TypeOf(req) != reflect.PtrTo(reflect.TypeOf(sc.srvReq)) {
		panic("wrong req")
	}
	if reflect.TypeOf(res) != reflect.PtrTo(reflect.TypeOf(sc.srvRes)) {
		panic("wrong res")
	}

	connCreatedInThisCall := false
	if sc.conn == nil {
		err := sc.createConn(ctx)
		if err != nil {
			return err
		}
		connCreatedInThisCall = true
	}

	connCopy := sc.conn
	funcDone := make(chan struct{})
	go func() {
		select {
		case <-ctx.Done():
			connCopy.Close()

		case <-funcDone:
			return
		}
	}()
	defer close(funcDone)

	err := sc.conn.WriteMessage(req)
	if err != nil {
		sc.conn.Close()
		sc.conn = nil

		// if the connection was created previously, it could be damaged or
		// linked to an invalid provider.
		// do another try.
		if !connCreatedInThisCall {
			return sc.CallContext(ctx, req, res)
		}

		return err
	}

	state, err := sc.conn.ReadServiceResponse(res)
	if err != nil {
		sc.conn.Close()
		sc.conn = nil

		// if the connection was created previously, it could be damaged or
		// linked to an invalid provider.
		// do another try.
		if !connCreatedInThisCall {
			return sc.CallContext(ctx, req, res)
		}

		return err
	}

	if !state {
		sc.conn.Close()
		sc.conn = nil
		return fmt.Errorf("service returned a failure state")
	}

	return nil
}

func (sc *ServiceClient) createConn(ctx context.Context) error {
	ur, err := sc.conf.Node.apiMasterClient.LookupService(
		sc.conf.Node.absoluteTopicName(sc.conf.Name))
	if err != nil {
		return fmt.Errorf("lookupService: %v", err)
	}

	address, err := urlToAddress(ur)
	if err != nil {
		return err
	}

	conn, err := prototcp.NewClientContext(ctx, address)
	if err != nil {
		return err
	}

	if sc.conf.EnableKeepAlive {
		conn.NetConn().(*net.TCPConn).SetKeepAlive(true)
		conn.NetConn().(*net.TCPConn).SetKeepAlivePeriod(60 * time.Second)
	}

	err = conn.WriteHeader(&prototcp.HeaderServiceClient{
		Callerid:   sc.conf.Node.absoluteName(),
		Md5sum:     sc.srvMD5,
		Persistent: 1,
		Service:    sc.conf.Node.absoluteTopicName(sc.conf.Name),
	})
	if err != nil {
		conn.Close()
		return err
	}

	raw, err := conn.ReadHeaderRaw()
	if err != nil {
		conn.Close()
		return err
	}

	if strErr, ok := raw["error"]; ok {
		conn.Close()
		return fmt.Errorf(strErr)
	}

	var outHeader prototcp.HeaderServiceProvider
	err = protocommon.HeaderDecode(raw, &outHeader)
	if err != nil {
		conn.Close()
		return err
	}

	if outHeader.Md5sum != sc.srvMD5 {
		conn.Close()
		return fmt.Errorf("wrong message checksum: expected %s, got %s",
			sc.srvMD5, outHeader.Md5sum)
	}

	sc.conn = conn
	return nil
}
