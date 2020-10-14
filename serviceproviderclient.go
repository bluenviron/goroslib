package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerid string
	client   *prototcp.Conn

	done chan struct{}
}

func newServiceProviderClient(sp *ServiceProvider, callerid string, client *prototcp.Conn) *serviceProviderClient {
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
	defer close(spc.done)

outer:
	for {
		req := reflect.New(spc.sp.reqMsg).Interface()
		err := spc.client.ReadMessage(req)
		if err != nil {
			break outer
		}

		spc.sp.clientRequest <- serviceProviderClientRequestReq{
			callerid: spc.callerid,
			req:      req,
		}
	}

	spc.sp.clientClose <- spc
}
