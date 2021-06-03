package apiparam

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestClient(t *testing.T) {
	s, err := xmlrpc.NewServer("localhost:9998")
	require.NoError(t, err)
	defer s.Close()

	go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
		switch raw.Method {
		case "getParamNames":
			return ResponseGetParamNames{Code: 1, List: []string{"val1", "val2"}}

		case "deleteParam":
			return ResponseDeleteParam{Code: 1}

		case "getParam":
			var req RequestGetParam
			err := raw.Decode(&req)
			require.NoError(t, err)

			switch req.Key {
			case "mykey1":
				return ResponseGetParamBool{Code: 1, Res: true}

			case "mykey2":
				return ResponseGetParamInt{Code: 1, Res: 123}

			case "mykey3":
				return ResponseGetParamString{Code: 1, Res: "mystring"}
			}

		case "hasParam":
			return ResponseHasParam{Code: 1, KeyOut: "mykey", Res: true}

		case "searchParam":
			return ResponseSearchParam{Code: 1, FoundKey: "mykey"}

		case "setParam":
			return ResponseSetParam{Code: 1}
		}
		return xmlrpc.ErrorRes{}
	})

	c := NewClient("localhost:9998", "test")

	func() {
		err := c.DeleteParam("mykey")
		require.NoError(t, err)
	}()

	func() {
		res, err := c.GetParamNames()
		require.NoError(t, err)
		require.Equal(t, []string{"val1", "val2"}, res)
	}()

	func() {
		res, err := c.GetParamBool("mykey1")
		require.NoError(t, err)
		require.Equal(t, true, res)
	}()

	func() {
		res, err := c.GetParamInt("mykey2")
		require.NoError(t, err)
		require.Equal(t, 123, res)
	}()

	func() {
		res, err := c.GetParamString("mykey3")
		require.NoError(t, err)
		require.Equal(t, "mystring", res)
	}()

	func() {
		res, err := c.HasParam("mykey")
		require.NoError(t, err)
		require.Equal(t, true, res)
	}()

	func() {
		res, err := c.SearchParam("mykey")
		require.NoError(t, err)
		require.Equal(t, "mykey", res)
	}()

	func() {
		err := c.SetParamBool("mykey", true)
		require.NoError(t, err)
	}()

	func() {
		err := c.SetParamInt("mykey", 123)
		require.NoError(t, err)
	}()

	func() {
		err := c.SetParamString("mykey", "myval")
		require.NoError(t, err)
	}()
}
