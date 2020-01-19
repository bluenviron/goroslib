package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/tcpros"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerid string
	client   *tcpros.Conn

	chanDone chan struct{}
}

func newServiceProviderClient(sp *ServiceProvider, callerid string, client *tcpros.Conn) *serviceProviderClient {
	spc := &serviceProviderClient{
		sp:       sp,
		callerid: callerid,
		client:   client,
		chanDone: make(chan struct{}),
	}

	go spc.run()

	return spc
}

func (sp *serviceProviderClient) close() {
	sp.client.Close()
	<-sp.chanDone
}

func (spc *serviceProviderClient) run() {
	defer func() { spc.chanDone <- struct{}{} }()

outer:
	for {
		req := reflect.New(spc.sp.reqType).Interface()
		err := spc.client.ReadMessage(req)
		if err != nil {
			break outer
		}

		spc.sp.chanEvents <- serviceProviderEventRequest{
			callerid: spc.callerid,
			req:      req,
		}
	}

	spc.client.Close()

	spc.sp.chanEvents <- serviceProviderEventClientClose{spc}
}
