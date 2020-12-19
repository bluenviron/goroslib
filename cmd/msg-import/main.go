package main

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"strings"
	"text/template"

	"gopkg.in/alecthomas/kingpin.v2"

	"github.com/aler9/goroslib/pkg/msgconv"
)

var tpl = template.Must(template.New("").Parse(
	`package {{ .GoPkgName }} //nolint:golint

import (
{{- range $k, $v := .Imports }}
    "{{ $k }}"
{{- end }}
)
{{ .Message }}
`))

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

	return ioutil.ReadAll(res.Body)
}

func run() error {
	kingpin.CommandLine.Help = "Convert ROS messages into Go structs."

	argGoPkgName := kingpin.Flag("gopackage", "Go package name").Default("main").String()
	argRosPkgName := kingpin.Flag("rospackage", "ROS package name").Default("my_package").String()
	argURL := kingpin.Arg("url", "path or url pointing to a ROS message").Required().String()

	kingpin.Parse()

	goPkgName := *argGoPkgName
	rosPkgName := *argRosPkgName
	u := *argURL

	isRemote := strings.HasPrefix(u, "https://") || strings.HasPrefix(u, "http://")

	content, err := func() (string, error) {
		if isRemote {
			byts, err := download(u)
			if err != nil {
				return "", err
			}
			return string(byts), nil
		}

		byts, err := ioutil.ReadFile(u)
		if err != nil {
			return "", err
		}
		return string(byts), nil
	}()
	if err != nil {
		return err
	}

	name := func() string {
		if isRemote {
			ur, _ := url.Parse(u)
			return strings.TrimSuffix(filepath.Base(ur.Path), ".msg")
		}
		return strings.TrimSuffix(filepath.Base(u), ".msg")
	}()

	msgDef, err := msgconv.ParseMessageDefinition(goPkgName, rosPkgName, name, content)
	if err != nil {
		return err
	}

	message, err := msgDef.Write()
	if err != nil {
		return err
	}

	return tpl.Execute(os.Stdout, map[string]interface{}{
		"GoPkgName":  goPkgName,
		"RosPkgName": rosPkgName,
		"Imports":    msgDef.Imports,
		"Message":    message,
	})
}

func main() {
	err := run()
	if err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
