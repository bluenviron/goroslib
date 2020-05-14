package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/tcpros"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerid string
	client   *tcpros.Conn

	done chan struct{}
}

func newServiceProviderClient(sp *ServiceProvider, callerid string, client *tcpros.Conn) *serviceProviderClient {
	spc := &serviceProviderClient{
		sp:       sp,
		callerid: callerid,
		client:   client,
		done:     make(chan struct{}),
	}

	go spc.run()

	return spc
}

func (sp *serviceProviderClient) close() {
	sp.client.Close()
	<-sp.done
}

func (spc *serviceProviderClient) run() {
	defer close(spc.done)

outer:
	for {
		req := reflect.New(spc.sp.reqMsg).Interface()
		err := spc.client.ReadMessage(req)
		if err != nil {
			break outer
		}

		spc.sp.events <- serviceProviderEventRequest{
			callerid: spc.callerid,
			req:      req,
		}
	}

	spc.client.Close()

	spc.sp.events <- serviceProviderEventClientClose{spc}
}
