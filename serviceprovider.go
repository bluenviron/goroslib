package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/apimaster"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClientNewReq struct {
	client *prototcp.Conn
	header *prototcp.HeaderServiceClient
}

type serviceProviderClientRequestReq struct {
	callerid string
	req      interface{}
}

// ServiceProviderConf is the configuration of a ServiceProvider.
type ServiceProviderConf struct {
	// node which the service provider belongs to
	Node *Node

	// name of the service from which providers will be obtained
	Service string

	// function in the form func(*NameOfRequest) *NameOfReply{}  that will be called
	// whenever a request arrives
	Callback interface{}
}

// ServiceProvider is a ROS service provider, an entity that can receive requests
// and send back replies.
type ServiceProvider struct {
	conf    ServiceProviderConf
	reqMsg  reflect.Type
	resMsg  reflect.Type
	reqType string
	resType string
	srvMd5  string
	clients map[string]*serviceProviderClient

	clientNew     chan serviceProviderClientNewReq
	clientClose   chan *serviceProviderClient
	clientRequest chan serviceProviderClientRequestReq
	shutdown      chan struct{}
	terminate     chan struct{}
	nodeTerminate chan struct{}
	done          chan struct{}
}

// NewServiceProvider allocates a ServiceProvider. See ServiceProviderConf for the options.
func NewServiceProvider(conf ServiceProviderConf) (*ServiceProvider, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if len(conf.Service) < 1 || conf.Service[0] != '/' {
		return nil, fmt.Errorf("Service must begin with /")
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

	reqMsg := cbt.In(0)
	if reqMsg.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Request must be a pointer")
	}
	if reqMsg.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Request must be a pointer to a struct")
	}

	resMsg := cbt.Out(0)
	if resMsg.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Response must be a pointer")
	}
	if resMsg.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Response must be a pointer to a struct")
	}

	reqType, err := msg.Type(reflect.New(reqMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	resType, err := msg.Type(reflect.New(resMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	srvMd5, err := msg.Md5Service(
		reflect.New(reqMsg.Elem()).Interface(),
		reflect.New(resMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	sp := &ServiceProvider{
		conf:          conf,
		reqMsg:        reqMsg.Elem(),
		resMsg:        resMsg.Elem(),
		reqType:       reqType,
		resType:       resType,
		srvMd5:        srvMd5,
		clients:       make(map[string]*serviceProviderClient),
		clientNew:     make(chan serviceProviderClientNewReq),
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
				req.client.Close()
				continue
			}

			if req.header.Md5sum != sp.srvMd5 {
				req.client.Close()
				continue
			}

			err := req.client.WriteHeader(&prototcp.HeaderServiceProvider{
				Callerid:     sp.conf.Node.conf.Name,
				Md5sum:       sp.srvMd5,
				RequestType:  sp.reqType,
				ResponseType: sp.resType,
				Type:         "goroslib/Service",
			})
			if err != nil {
				req.client.Close()
				continue
			}

			sp.clients[req.header.Callerid] = newServiceProviderClient(sp, req.header.Callerid, req.client)

		case spc := <-sp.clientClose:
			delete(sp.clients, spc.callerid)
			spc.client.Close()

		case req := <-sp.clientRequest:
			res := cbv.Call([]reflect.Value{reflect.ValueOf(req.req)})

			conn, ok := sp.clients[req.callerid]
			if !ok {
				continue
			}

			err := conn.client.WriteServiceResState()
			if err != nil {
				continue
			}

			conn.client.WriteMessage(res[0].Interface())

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

	sp.conf.Node.apiMasterClient.UnregisterService(apimaster.RequestUnregisterService{
		Service:    sp.conf.Service[1:],
		ServiceUrl: sp.conf.Node.tcprosServerUrl,
	})

	for _, spc := range sp.clients {
		spc.client.Close()
		<-spc.done
	}

	sp.conf.Node.serviceProviderClose <- sp

	<-sp.nodeTerminate
	<-sp.terminate

	close(sp.clientNew)
	close(sp.clientClose)
	close(sp.clientRequest)
	close(sp.shutdown)
}
