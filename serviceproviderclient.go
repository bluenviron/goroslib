package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerId string
	client   *prototcp.Conn
}

func newServiceProviderClient(sp *ServiceProvider, callerId string, client *prototcp.Conn) {
	spc := &serviceProviderClient{
		sp:       sp,
		callerId: callerId,
		client:   client,
	}

	sp.clients[callerId] = spc

	sp.clientsWg.Add(1)
	go spc.run()
}

func (spc *serviceProviderClient) close() {
	delete(spc.sp.clients, spc.callerId)
	spc.client.Close()
}

func (spc *serviceProviderClient) run() {
	defer spc.sp.clientsWg.Done()

outer:
	for {
		req := reflect.New(spc.sp.reqMsg).Interface()
		err := spc.client.ReadMessage(req)
		if err != nil {
			break outer
		}

		spc.sp.clientRequest <- serviceProviderClientRequestReq{
			callerId: spc.callerId,
			req:      req,
		}
	}

	spc.sp.clientClose <- spc
}
