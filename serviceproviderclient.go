package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerID string
	client   *prototcp.Conn
}

func newServiceProviderClient(sp *ServiceProvider, callerID string, client *prototcp.Conn) {
	spc := &serviceProviderClient{
		sp:       sp,
		callerID: callerID,
		client:   client,
	}

	sp.clients[callerID] = spc

	sp.clientsWg.Add(1)
	go spc.run()
}

func (spc *serviceProviderClient) close() {
	delete(spc.sp.clients, spc.callerID)
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
			callerID: spc.callerID,
			req:      req,
		}
	}

	spc.sp.clientClose <- spc
}
