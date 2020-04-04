package goroslib

import (
	"net/url"
	"strconv"
)

func ptrString(v string) *string {
	return &v
}

func ptrInt(v int) *int {
	return &v
}

func parseUrl(in string) (string, uint16, error) {
	u, err := url.Parse(in)
	if err != nil {
		return "", 0, err
	}

	port, err := strconv.ParseUint(u.Port(), 10, 16)
	if err != nil {
		return "", 0, err
	}

	return u.Hostname(), uint16(port), nil
}
