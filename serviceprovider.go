package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/msg-utils"
	"github.com/aler9/goroslib/tcpros"
)

type serviceProviderEvent interface {
	isServiceProviderEvent()
}

type serviceProviderEventClose struct {
}

func (serviceProviderEventClose) isServiceProviderEvent() {}

type serviceProviderEventClientNew struct {
	client *tcpros.Conn
	header *tcpros.HeaderServiceClient
}

func (serviceProviderEventClientNew) isServiceProviderEvent() {}

type serviceProviderEventClientClose struct {
	spc *serviceProviderClient
}

func (serviceProviderEventClientClose) isServiceProviderEvent() {}

type serviceProviderEventRequest struct {
	callerid string
	req      interface{}
}

func (serviceProviderEventRequest) isServiceProviderEvent() {}

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

	events    chan serviceProviderEvent
	terminate chan struct{}
	done      chan struct{}

	clients map[string]*serviceProviderClient
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

	reqType, err := msg_utils.MessageType(reflect.New(reqMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	resType, err := msg_utils.MessageType(reflect.New(resMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	srvMd5, err := msg_utils.ServiceMd5(
		reflect.New(reqMsg.Elem()).Interface(),
		reflect.New(resMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	sp := &ServiceProvider{
		conf:      conf,
		reqMsg:    reqMsg.Elem(),
		resMsg:    resMsg.Elem(),
		reqType:   reqType,
		resType:   resType,
		srvMd5:    srvMd5,
		events:    make(chan serviceProviderEvent),
		terminate: make(chan struct{}),
		done:      make(chan struct{}),
		clients:   make(map[string]*serviceProviderClient),
	}

	errored := make(chan error)
	conf.Node.events <- nodeEventServiceProviderNew{
		sp:  sp,
		err: errored,
	}
	err = <-errored
	if err != nil {
		return nil, err
	}

	go sp.run()

	return sp, nil
}

func (sp *ServiceProvider) run() {
	cbv := reflect.ValueOf(sp.conf.Callback)

outer:
	for {
		rawEvt := <-sp.events
		switch evt := rawEvt.(type) {
		case serviceProviderEventClientNew:
			_, ok := sp.clients[evt.header.Callerid]
			if ok {
				evt.client.Close()
				continue
			}

			if evt.header.Md5sum != sp.srvMd5 {
				evt.client.Close()
				continue
			}

			err := evt.client.WriteHeader(&tcpros.HeaderServiceProvider{
				Callerid:     ptrString(sp.conf.Node.conf.Name),
				Md5sum:       ptrString(sp.srvMd5),
				RequestType:  ptrString(sp.reqType),
				ResponseType: ptrString(sp.resType),
				Type:         ptrString("goroslib/Service"),
			})
			if err != nil {
				evt.client.Close()
				continue
			}

			sp.clients[evt.header.Callerid] = newServiceProviderClient(sp, evt.header.Callerid, evt.client)

		case serviceProviderEventClientClose:
			delete(sp.clients, evt.spc.callerid)

		case serviceProviderEventRequest:
			res := cbv.Call([]reflect.Value{reflect.ValueOf(evt.req)})

			conn, ok := sp.clients[evt.callerid]
			if !ok {
				continue
			}

			err := conn.client.WriteServiceResState()
			if err != nil {
				continue
			}

			conn.client.WriteMessage(res[0].Interface())

		case serviceProviderEventClose:
			break outer
		}
	}

	// consume queue
	go func() {
		for range sp.events {
		}
	}()

	sp.conf.Node.apiMasterClient.UnregisterService(api_master.RequestUnregisterService{
		Service:    sp.conf.Service[1:],
		ServiceUrl: sp.conf.Node.tcprosServer.GetUrl(),
	})

	for _, c := range sp.clients {
		c.close()
	}

	done := make(chan struct{})
	sp.conf.Node.events <- nodeEventServiceProviderClose{sp, done}
	<-done

	// wait Close()
	<-sp.terminate

	close(sp.events)

	close(sp.done)
}

// Close closes a ServiceProvider and shuts down all its operations.
func (sp *ServiceProvider) Close() error {
	sp.events <- serviceProviderEventClose{}
	close(sp.terminate)
	<-sp.done
	return nil
}
