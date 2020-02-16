package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msg-utils"
	"github.com/aler9/goroslib/tcpros"
)

type serviceProviderEvent interface {
}

type serviceProviderEventClose struct {
}

type serviceProviderEventClientNew struct {
	client *tcpros.Conn
	header *tcpros.HeaderServiceClient
}

type serviceProviderEventClientClose struct {
	spc *serviceProviderClient
}

type serviceProviderEventRequest struct {
	callerid string
	req      interface{}
}

type ServiceProviderConf struct {
	Node     *Node
	Service  string
	Callback interface{}
}

type ServiceProvider struct {
	conf    ServiceProviderConf
	reqType reflect.Type
	resType reflect.Type
	srvMd5  string

	chanEvents chan serviceProviderEvent
	chanDone   chan struct{}

	clients map[string]*serviceProviderClient
}

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

	reqType := cbt.In(0)
	if reqType.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Request must be a pointer")
	}
	if reqType.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Request must be a pointer to a struct")
	}

	resType := cbt.Out(0)
	if resType.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Response must be a pointer")
	}
	if resType.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Response must be a pointer to a struct")
	}

	srvMd5, err := msg_utils.ServiceMd5(
		reflect.New(reqType.Elem()).Interface(),
		reflect.New(resType.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	sp := &ServiceProvider{
		conf:       conf,
		reqType:    reqType.Elem(),
		resType:    resType.Elem(),
		srvMd5:     srvMd5,
		chanEvents: make(chan serviceProviderEvent),
		chanDone:   make(chan struct{}),
		clients:    make(map[string]*serviceProviderClient),
	}

	chanErr := make(chan error)
	conf.Node.chanEvents <- nodeEventServiceProviderNew{
		sp:      sp,
		chanErr: chanErr,
	}
	err = <-chanErr
	if err != nil {
		return nil, err
	}

	go sp.run()

	return sp, nil
}

func (sp *ServiceProvider) Close() error {
	sp.chanEvents <- serviceProviderEventClose{}
	<-sp.chanDone
	return nil
}

func (sp *ServiceProvider) run() {
	defer func() { sp.chanDone <- struct{}{} }()

	cbv := reflect.ValueOf(sp.conf.Callback)

outer:
	for {
		rawEvt := <-sp.chanEvents
		switch evt := rawEvt.(type) {
		case serviceProviderEventClose:
			break outer

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
				Callerid:     sp.conf.Node.conf.Name,
				Md5sum:       sp.srvMd5,
				RequestType:  "goroslib/Msg",
				ResponseType: "goroslib/Msg",
				Type:         "goroslib/Msg",
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
		}
	}

	// consume queue
	go func() {
		for range sp.chanEvents {
		}
	}()

	for _, c := range sp.clients {
		c.close()
	}

	sp.conf.Node.chanEvents <- nodeEventServiceProviderClose{sp}

	close(sp.chanEvents)
}
