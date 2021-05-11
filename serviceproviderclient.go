package goroslib

import (
	"reflect"

	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerID string
	conn     *prototcp.Conn
}

func newServiceProviderClient(sp *ServiceProvider, callerID string, conn *prototcp.Conn) {
	spc := &serviceProviderClient{
		sp:       sp,
		callerID: callerID,
		conn:     conn,
	}

	sp.clients[callerID] = spc

	sp.clientsWg.Add(1)
	go spc.run()
}

func (spc *serviceProviderClient) close() {
	delete(spc.sp.clients, spc.callerID)
	spc.conn.Close()
}

func (spc *serviceProviderClient) run() {
	defer spc.sp.clientsWg.Done()

outer:
	for {
		req := reflect.New(reflect.TypeOf(spc.sp.srvReq)).Interface()
		err := spc.conn.ReadMessage(req)
		if err != nil {
			break outer
		}

		select {
		case spc.sp.clientRequest <- serviceProviderClientRequestReq{
			callerID: spc.callerID,
			req:      req,
		}:
		case <-spc.sp.ctx.Done():
		}
	}

	select {
	case spc.sp.clientClose <- spc:
	case <-spc.sp.ctx.Done():
	}
}
