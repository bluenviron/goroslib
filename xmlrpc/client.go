package xmlrpc

import (
	"bytes"
	"fmt"
	"net/http"
)

func Client(url string, method string, paramsReq interface{}, paramsRes interface{}) error {
	if url[len(url)-1] != '/' {
		return fmt.Errorf("url must end with /")
	}

	var buf bytes.Buffer
	err := requestEncode(&buf, method, paramsReq)
	if err != nil {
		return err
	}

	res, err := http.Post(url+"RPC2", "text/xml", &buf)
	if err != nil {
		return err
	}
	defer res.Body.Close()

	err = responseDecode(res.Body, paramsRes)
	if err != nil {
		return err
	}

	return nil
}
