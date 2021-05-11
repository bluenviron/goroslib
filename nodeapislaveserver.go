package goroslib

import (
	"os"
	"sync"

	"github.com/aler9/goroslib/pkg/apislave"
)

func (n *Node) runAPISlaveServer(wg *sync.WaitGroup) {
	defer wg.Done()

	n.apiSlaveServer.Handle(n.handleAPISlaveServer)
}

func (n *Node) handleAPISlaveServer(req apislave.Request) apislave.Response {
	switch reqt := req.(type) {
	case *apislave.RequestGetBusInfo:
		res := make(chan apislave.ResponseGetBusInfo)
		select {
		case n.getBusInfo <- getBusInfoReq{res}:
			return <-res

		case <-n.ctx.Done():
			return apislave.ResponseGetBusInfo{
				Code:          0,
				StatusMessage: "terminating",
			}
		}

	case *apislave.RequestGetPid:
		return apislave.ResponseGetPid{
			Code:          1,
			StatusMessage: "",
			Pid:           os.Getpid(),
		}

	case *apislave.RequestGetPublications:
		res := make(chan [][]string)

		select {
		case n.getPublications <- getPublicationsReq{res}:
			return apislave.ResponseGetPublications{
				Code:          1,
				StatusMessage: "",
				TopicList:     <-res,
			}

		case <-n.ctx.Done():
			return apislave.ResponseGetPublications{
				Code:          1,
				StatusMessage: "",
				TopicList:     [][]string{},
			}
		}

	case *apislave.RequestPublisherUpdate:
		select {
		case n.subscriberPubUpdate <- subscriberPubUpdateReq{
			topic: reqt.Topic,
			urls:  reqt.PublisherURLs,
		}:
		case <-n.ctx.Done():
		}

		return apislave.ResponsePublisherUpdate{
			Code:          1,
			StatusMessage: "",
		}

	case *apislave.RequestRequestTopic:
		res := make(chan apislave.ResponseRequestTopic)
		select {
		case n.subscriberRequestTopic <- subscriberRequestTopicReq{reqt, res}:
			return <-res

		case <-n.ctx.Done():
			return apislave.ResponseRequestTopic{
				Code:          0,
				StatusMessage: "terminated",
			}
		}

	case *apislave.RequestShutdown:
		n.ctxCancel()

		return apislave.ResponseShutdown{
			Code:          1,
			StatusMessage: "",
		}
	}

	return apislave.ErrorRes{}
}
