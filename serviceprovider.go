package goroslib

import (
	"context"
	"fmt"
	"reflect"
	"sync"

	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/serviceproc"
)

type serviceProviderClientRequestReq struct {
	callerID string
	req      interface{}
}

// ServiceProviderConf is the configuration of a ServiceProvider.
type ServiceProviderConf struct {
	// parent node.
	Node *Node

	// name of the service.
	Name string

	// an instance of the service type.
	Srv interface{}

	// function in the form func(*NameOfRequest) *NameOfReply{}  that will be called
	// whenever a request arrives.
	Callback interface{}
}

// ServiceProvider is a ROS service provider, an entity that can receive requests
// and send back responses.
type ServiceProvider struct {
	conf ServiceProviderConf

	ctx       context.Context
	ctxCancel func()
	srvType   string
	srvMD5    string
	srvReq    interface{}
	clients   map[string]*serviceProviderClient
	clientsWg sync.WaitGroup

	// in
	clientNew     chan tcpConnServiceClientReq
	clientClose   chan *serviceProviderClient
	clientRequest chan serviceProviderClientRequestReq

	// out
	done chan struct{}
}

// NewServiceProvider allocates a ServiceProvider. See ServiceProviderConf for the options.
func NewServiceProvider(conf ServiceProviderConf) (*ServiceProvider, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Name == "" {
		return nil, fmt.Errorf("Name is empty")
	}

	if conf.Srv == nil {
		return nil, fmt.Errorf("Srv is empty")
	}

	srvType, err := serviceproc.Type(conf.Srv)
	if err != nil {
		return nil, err
	}

	srvMD5, err := serviceproc.MD5(conf.Srv)
	if err != nil {
		return nil, err
	}

	srvReq, srvRes, err := serviceproc.RequestResponse(conf.Srv)
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

	ctx, ctxCancel := context.WithCancel(conf.Node.ctx)

	sp := &ServiceProvider{
		conf:          conf,
		ctx:           ctx,
		ctxCancel:     ctxCancel,
		srvType:       srvType,
		srvMD5:        srvMD5,
		srvReq:        srvReq,
		clients:       make(map[string]*serviceProviderClient),
		clientNew:     make(chan tcpConnServiceClientReq),
		clientClose:   make(chan *serviceProviderClient),
		clientRequest: make(chan serviceProviderClientRequestReq),
		done:          make(chan struct{}),
	}

	cerr := make(chan error)
	select {
	case conf.Node.serviceProviderNew <- serviceProviderNewReq{
		sp:  sp,
		err: cerr,
	}:
		err = <-cerr
		if err != nil {
			return nil, err
		}

	case <-sp.ctx.Done():
		return nil, fmt.Errorf("terminated")
	}

	go sp.run()

	return sp, nil
}

// Close closes a ServiceProvider and shuts down all its operations.
func (sp *ServiceProvider) Close() error {
	sp.ctxCancel()
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

			err := client.conn.WriteServiceResState(1)
			if err != nil {
				continue
			}

			client.conn.WriteMessage(res[0].Interface())

		case <-sp.ctx.Done():
			break outer
		}
	}

	sp.ctxCancel()

	sp.conf.Node.apiMasterClient.UnregisterService(
		sp.conf.Node.absoluteTopicName(sp.conf.Name),
		sp.conf.Node.tcprosServerURL)

	sp.clientsWg.Wait()

	select {
	case sp.conf.Node.serviceProviderClose <- sp:
	case <-sp.conf.Node.ctx.Done():
	}
}
