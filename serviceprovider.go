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
	spc *serviceProviderClient
	req interface{}
}

// ServiceProviderConf is the configuration of a ServiceProvider.
type ServiceProviderConf struct {
	// parent node.
	Node *Node

	// name of the service.
	Name string

	// an instance of the service type.
	Srv interface{}

	// function in the form func(*NameOfRequest) (*NameOfReply{}, bool)
	// that will be called when a request arrives.
	Callback interface{}

	onClient func()
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
	clients   map[*serviceProviderClient]struct{}
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

	if reflect.TypeOf(conf.Srv).Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Srv is not a pointer")
	}

	srvElem := reflect.ValueOf(conf.Srv).Elem().Interface()

	srvReq, srvRes, err := serviceproc.RequestResponse(srvElem)
	if err != nil {
		return nil, err
	}

	srvType, err := serviceproc.Type(srvElem)
	if err != nil {
		return nil, err
	}

	srvMD5, err := serviceproc.MD5(srvElem)
	if err != nil {
		return nil, err
	}

	conf.Name = conf.Node.applyCliRemapping(conf.Name)

	cbt := reflect.TypeOf(conf.Callback)
	if cbt.Kind() != reflect.Func {
		return nil, fmt.Errorf("Callback is not a function")
	}

	if cbt.NumIn() != 1 {
		return nil, fmt.Errorf("Callback must accept a single argument")
	}
	vin := cbt.In(0)
	if vin.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("argument must be a pointer")
	}
	if vin.Elem() != reflect.TypeOf(srvReq) {
		return nil, fmt.Errorf("invalid callback argument")
	}

	if cbt.NumOut() != 2 {
		return nil, fmt.Errorf("Callback must return 2 values")
	}
	vout := cbt.Out(0)
	if vout.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("first return value must be a pointer")
	}
	if vout.Elem() != reflect.TypeOf(srvRes) {
		return nil, fmt.Errorf("invalid callback return value")
	}
	vout = cbt.Out(1)
	if vout.Kind() != reflect.Bool {
		return nil, fmt.Errorf("second return value must be a bool")
	}

	ctx, ctxCancel := context.WithCancel(conf.Node.ctx)

	sp := &ServiceProvider{
		conf:          conf,
		ctx:           ctx,
		ctxCancel:     ctxCancel,
		srvType:       srvType,
		srvMD5:        srvMD5,
		srvReq:        srvReq,
		clients:       make(map[*serviceProviderClient]struct{}),
		clientNew:     make(chan tcpConnServiceClientReq),
		clientClose:   make(chan *serviceProviderClient),
		clientRequest: make(chan serviceProviderClientRequestReq),
		done:          make(chan struct{}),
	}

	sp.conf.Node.Log(LogLevelDebug, "service provider '%s' created", sp.conf.Node.absoluteTopicName(sp.conf.Name))

	cerr := make(chan error)
	select {
	case conf.Node.serviceProviderNew <- serviceProviderNewReq{sp: sp, res: cerr}:
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

	sp.conf.Node.Log(LogLevelDebug, "service provider '%s' destroyed", sp.conf.Node.absoluteTopicName(sp.conf.Name))
	return nil
}

func (sp *ServiceProvider) run() {
	defer close(sp.done)

	cbv := reflect.ValueOf(sp.conf.Callback)

outer:
	for {
		select {
		case req := <-sp.clientNew:
			err := func() error {
				if req.header.Md5sum != "*" && req.header.Md5sum != sp.srvMD5 {
					return fmt.Errorf("wrong service checksum: expected %s, got %s",
						sp.srvMD5, req.header.Md5sum)
				}

				return req.conn.WriteHeader(&prototcp.HeaderServiceProvider{
					Callerid:     sp.conf.Node.absoluteName(),
					Md5sum:       sp.srvMD5,
					RequestType:  sp.srvType + "Request",
					ResponseType: sp.srvType + "Response",
					Type:         sp.srvType,
				})
			}()
			if err != nil {
				sp.conf.Node.Log(LogLevelError,
					"service provider '%s' is unable to accept client '%s': %s",
					sp.conf.Node.absoluteTopicName(sp.conf.Name),
					req.conn.NetConn().RemoteAddr(),
					err)
				req.conn.Close()
				continue
			}

			spc := newServiceProviderClient(sp, req.header.Callerid, req.conn)
			sp.clients[spc] = struct{}{}

		case spc := <-sp.clientClose:
			delete(sp.clients, spc)

		case req := <-sp.clientRequest:
			res := cbv.Call([]reflect.Value{reflect.ValueOf(req.req)})

			state := res[1].Interface().(bool)
			msg := res[0].Interface()

			err := req.spc.conn.WriteServiceResponse(state, msg)
			if err != nil {
				sp.conf.Node.Log(LogLevelError,
					"service provider '%s' is unable to write to client '%s': %s",
					sp.conf.Node.absoluteTopicName(sp.conf.Name),
					req.spc.conn.NetConn().RemoteAddr(),
					err)
				continue
			}

		case <-sp.ctx.Done():
			break outer
		}
	}

	sp.ctxCancel()

	sp.conf.Node.apiMasterClient.UnregisterService(
		sp.conf.Node.absoluteTopicName(sp.conf.Name),
		sp.conf.Node.tcprosServer.URL())

	sp.clientsWg.Wait()

	select {
	case sp.conf.Node.serviceProviderClose <- sp:
	case <-sp.conf.Node.ctx.Done():
	}
}
