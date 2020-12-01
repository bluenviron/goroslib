package goroslib

import (
	"net/url"
)

func urlToAddress(in string) (string, error) {
	u, err := url.Parse(in)
	if err != nil {
		return "", err
	}

	return u.Host, nil
}
