package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/proto-tcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerid string
	client   *proto_tcp.Conn

	done chan struct{}
}

func newServiceProviderClient(sp *ServiceProvider, callerid string, client *proto_tcp.Conn) *serviceProviderClient {
	spc := &serviceProviderClient{
		sp:       sp,
		callerid: callerid,
		client:   client,
		done:     make(chan struct{}),
	}

	go spc.run()

	return spc
}

func (spc *serviceProviderClient) run() {
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

	close(spc.done)
}

func (sp *serviceProviderClient) close() {
	sp.client.Close()
	<-sp.done
}
