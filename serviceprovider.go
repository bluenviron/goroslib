package goroslib

import (
	"fmt"
	"reflect"
	"sync"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClientRequestReq struct {
	callerID string
	req      interface{}
}

// ServiceProviderConf is the configuration of a ServiceProvider.
type ServiceProviderConf struct {
	// node which the service provider belongs to
	Node *Node

	// name of the service
	Name string

	// an instance of the service type, which is a struct containing
	// - goroslib.Package
	// - a request
	// - a response
	Srv interface{}

	// function in the form func(*NameOfRequest) *NameOfReply{}  that will be called
	// whenever a request arrives
	Callback interface{}
}

// ServiceProvider is a ROS service provider, an entity that can receive requests
// and send back responses.
type ServiceProvider struct {
	conf      ServiceProviderConf
	srvType   string
	srvMD5    string
	srvReq    interface{}
	clients   map[string]*serviceProviderClient
	clientsWg sync.WaitGroup

	// in
	clientNew     chan tcpConnServiceClientReq
	clientClose   chan *serviceProviderClient
	clientRequest chan serviceProviderClientRequestReq
	shutdown      chan struct{}
	terminate     chan struct{}
	nodeTerminate chan struct{}

	// out
	done chan struct{}
}

// NewServiceProvider allocates a ServiceProvider. See ServiceProviderConf for the options.
func NewServiceProvider(conf ServiceProviderConf) (*ServiceProvider, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Srv == nil {
		return nil, fmt.Errorf("Srv is empty")
	}

	srvType, err := msg.ServiceType(conf.Srv)
	if err != nil {
		return nil, err
	}

	srvMD5, err := msg.ServiceMD5(conf.Srv)
	if err != nil {
		return nil, err
	}

	srvReq, srvRes, err := msg.ServiceRequestResponse(conf.Srv)
	if err != nil {
		return nil, err
	}

	cbt := reflect.TypeOf(conf.Callback)
	if cbt.Kind() != reflect.Func {
		return nil, fmt.Errorf("Callback is not a function")
	}
	if cbt.NumIn() != 1 {
		return nil, fmt.Errorf("Callback must accept a single argument")
	}
	if cbt.NumOut() != 1 {
		return nil, fmt.Errorf("Callback must return a single argument")
	}

	cbIn := cbt.In(0)
	if cbIn.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("argument must be a pointer")
	}
	if cbIn.Elem() != reflect.TypeOf(srvReq) {
		return nil, fmt.Errorf("invalid callback argument")
	}

	cbOut := cbt.Out(0)
	if cbOut.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("return value must be a pointer")
	}
	if cbOut.Elem() != reflect.TypeOf(srvRes) {
		return nil, fmt.Errorf("invalid callback return value")
	}

	sp := &ServiceProvider{
		conf:          conf,
		srvType:       srvType,
		srvMD5:        srvMD5,
		srvReq:        srvReq,
		clients:       make(map[string]*serviceProviderClient),
		clientNew:     make(chan tcpConnServiceClientReq),
		clientClose:   make(chan *serviceProviderClient),
		clientRequest: make(chan serviceProviderClientRequestReq),
		shutdown:      make(chan struct{}),
		terminate:     make(chan struct{}),
		nodeTerminate: make(chan struct{}),
		done:          make(chan struct{}),
	}

	chanErr := make(chan error)
	conf.Node.serviceProviderNew <- serviceProviderNewReq{
		sp:  sp,
		err: chanErr,
	}
	err = <-chanErr
	if err != nil {
		return nil, err
	}

	go sp.run()

	return sp, nil
}

// Close closes a ServiceProvider and shuts down all its operations.
func (sp *ServiceProvider) Close() error {
	sp.shutdown <- struct{}{}
	close(sp.terminate)
	<-sp.done
	return nil
}

func (sp *ServiceProvider) run() {
	defer close(sp.done)

	cbv := reflect.ValueOf(sp.conf.Callback)

outer:
	for {
		select {
		case req := <-sp.clientNew:
			_, ok := sp.clients[req.header.Callerid]
			if ok {
				req.conn.Close()
				continue
			}

			if req.header.Md5sum != "*" && req.header.Md5sum != sp.srvMD5 {
				req.conn.Close()
				continue
			}

			err := req.conn.WriteHeader(&prototcp.HeaderServiceProvider{
				Callerid:     sp.conf.Node.absoluteName(),
				Md5sum:       sp.srvMD5,
				RequestType:  sp.srvType + "Request",
				ResponseType: sp.srvType + "Response",
				Type:         sp.srvType,
			})
			if err != nil {
				req.conn.Close()
				continue
			}

			newServiceProviderClient(sp, req.header.Callerid, req.conn)

		case spc := <-sp.clientClose:
			spc.close()

		case req := <-sp.clientRequest:
			res := cbv.Call([]reflect.Value{reflect.ValueOf(req.req)})

			client, ok := sp.clients[req.callerID]
			if !ok {
				continue
			}

			err := client.conn.WriteServiceResState()
			if err != nil {
				continue
			}

			client.conn.WriteMessage(res[0].Interface())

		case <-sp.shutdown:
			break outer
		}
	}

	go func() {
		for {
			select {
			case _, ok := <-sp.clientNew:
				if !ok {
					return
				}
			case <-sp.clientClose:
			case <-sp.clientRequest:
			case <-sp.shutdown:
			}
		}
	}()

	sp.conf.Node.apiMasterClient.UnregisterService(
		sp.conf.Node.absoluteTopicName(sp.conf.Name),
		sp.conf.Node.tcprosServerURL)

	for _, spc := range sp.clients {
		spc.close()
	}
	sp.clientsWg.Wait()

	sp.conf.Node.serviceProviderClose <- sp

	<-sp.nodeTerminate
	<-sp.terminate

	close(sp.clientNew)
	close(sp.clientClose)
	close(sp.clientRequest)
	close(sp.shutdown)
}
