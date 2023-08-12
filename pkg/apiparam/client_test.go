package apiparam

import (
	"net/http"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/xmlrpc"
)

func TestClient(t *testing.T) {
	s, err := xmlrpc.NewServer("localhost:9998", 5*time.Second)
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

			case "mykey4":
				return ResponseGetParamFloat64{Code: 1, Res: 123.456}
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

	c := NewClient("localhost:9998", "test", &http.Client{})

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
		res, err := c.GetParamFloat64("mykey4")
		require.NoError(t, err)
		require.Equal(t, 123.456, res)
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

	func() {
		err := c.SetParamFloat64("mykey", 127.6)
		require.NoError(t, err)
	}()
}

func TestClientError(t *testing.T) {
	c := NewClient("localhost:9998", "test", &http.Client{})

	t.Run("no server", func(t *testing.T) {
		func() {
			err := c.DeleteParam("mykey")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetParamNames()
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetParamBool("mykey1")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetParamInt("mykey2")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetParamString("mykey3")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetParamFloat64("mykey4")
			require.Error(t, err)
		}()

		func() {
			_, err := c.HasParam("mykey")
			require.Error(t, err)
		}()

		func() {
			_, err := c.SearchParam("mykey")
			require.Error(t, err)
		}()

		func() {
			err := c.SetParamBool("mykey", true)
			require.Error(t, err)
		}()

		func() {
			err := c.SetParamInt("mykey", 123)
			require.Error(t, err)
		}()

		func() {
			err := c.SetParamString("mykey", "myval")
			require.Error(t, err)
		}()
	})

	t.Run("server error", func(t *testing.T) {
		s, err := xmlrpc.NewServer("localhost:9998", 5*time.Second)
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
			switch raw.Method {
			case "getParamNames":
				return ResponseGetParamNames{Code: 0, List: []string{"val1", "val2"}}

			case "deleteParam":
				return ResponseDeleteParam{Code: 0}

			case "getParam":
				var req RequestGetParam
				err := raw.Decode(&req)
				require.NoError(t, err)

				switch req.Key {
				case "mykey1":
					return ResponseGetParamBool{Code: 0, Res: true}

				case "mykey2":
					return ResponseGetParamInt{Code: 0, Res: 123}

				case "mykey3":
					return ResponseGetParamString{Code: 0, Res: "mystring"}

				case "mykey4":
					return ResponseGetParamFloat64{Code: 0, Res: 123.456}
				}

			case "hasParam":
				var req RequestHasParam
				err := raw.Decode(&req)
				require.NoError(t, err)

				if req.Key == "mykey" {
					return ResponseHasParam{Code: 0, KeyOut: "mykey", Res: true}
				}

				return ResponseHasParam{Code: 1, KeyOut: "mykey_wrong", Res: true}

			case "searchParam":
				return ResponseSearchParam{Code: 0, FoundKey: "mykey"}

			case "setParam":
				return ResponseSetParam{Code: 0}
			}
			return xmlrpc.ErrorRes{}
		})

		err = c.DeleteParam("mykey")
		require.Error(t, err)

		_, err = c.GetParamNames()
		require.Error(t, err)

		_, err = c.GetParamBool("mykey1")
		require.Error(t, err)

		_, err = c.GetParamInt("mykey2")
		require.Error(t, err)

		_, err = c.GetParamString("mykey3")
		require.Error(t, err)

		_, err = c.GetParamFloat64("mykey4")
		require.Error(t, err)

		_, err = c.HasParam("mykey")
		require.Error(t, err)

		_, err = c.HasParam("mykey2")
		require.Error(t, err)

		_, err = c.SearchParam("mykey")
		require.Error(t, err)

		err = c.SetParamBool("mykey", true)
		require.Error(t, err)

		err = c.SetParamInt("mykey", 123)
		require.Error(t, err)

		err = c.SetParamString("mykey", "myval")
		require.Error(t, err)
	})
}
