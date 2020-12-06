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
		n.getBusInfo <- getBusInfoReq{res}
		return <-res

	case *apislave.RequestGetPid:
		return apislave.ResponseGetPid{
			Code:          1,
			StatusMessage: "",
			Pid:           os.Getpid(),
		}

	case *apislave.RequestGetPublications:
		res := make(chan [][]string)
		n.getPublications <- getPublicationsReq{res}
		return apislave.ResponseGetPublications{
			Code:          1,
			StatusMessage: "",
			TopicList:     <-res,
		}

	case *apislave.RequestPublisherUpdate:
		n.subscriberPubUpdate <- subscriberPubUpdateReq{
			topic: reqt.Topic,
			urls:  reqt.PublisherURLs,
		}

		return apislave.ResponsePublisherUpdate{
			Code:          1,
			StatusMessage: "",
		}

	case *apislave.RequestRequestTopic:
		res := make(chan apislave.ResponseRequestTopic)
		n.subscriberRequestTopic <- subscriberRequestTopicReq{reqt, res}
		return <-res

	case *apislave.RequestShutdown:
		n.shutdown <- struct{}{}

		return apislave.ResponseShutdown{
			Code:          1,
			StatusMessage: "",
		}
	}

	return apislave.ErrorRes{}
}
