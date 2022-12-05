package cmd

import (
	"io"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"strings"
)

func download(addr string) ([]byte, error) {
	req, err := http.NewRequest("GET", addr, nil)
	if err != nil {
		return nil, err
	}

	res, err := http.DefaultClient.Do(req)
	if err != nil {
		return nil, err
	}
	defer res.Body.Close()

	return io.ReadAll(res.Body)
}

func getContent(u string) (string, error) {
	if isRemote(u) {
		byts, err := download(u)
		if err != nil {
			return "", err
		}
		return string(byts), nil
	}

	byts, err := os.ReadFile(u)
	if err != nil {
		return "", err
	}
	return string(byts), nil
}

func getName(u string, ext string) string {
	if isRemote(u) {
		ur, _ := url.Parse(u)
		return strings.TrimSuffix(filepath.Base(ur.Path), ext)
	}
	return strings.TrimSuffix(filepath.Base(u), ext)
}

func isRemote(u string) bool {
	return strings.HasPrefix(u, "https://") || strings.HasPrefix(u, "http://")
}
