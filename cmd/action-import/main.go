// main package.
package main

import (
	"fmt"
	"io"
	"os"

	"github.com/alecthomas/kong"

	"github.com/bluenviron/goroslib/v2/pkg/conversion"
)

var cli struct {
	GoPackage  string `name:"gopackage" help:"Go package name" default:"main"`
	RosPackage string `name:"rospackage" help:"ROS package name" default:"my_package"`
	Path       string `arg:"" help:"path pointing to a ROS action"`
}

func run(args []string, output io.Writer) error {
	parser, err := kong.New(&cli,
		kong.Description("Convert ROS actions into Go structs."),
		kong.UsageOnError())
	if err != nil {
		return err
	}

	_, err = parser.Parse(args)
	if err != nil {
		return err
	}

	return conversion.ImportAction(cli.Path, cli.GoPackage, cli.RosPackage, output)
}

func main() {
	if err := run(os.Args[1:], os.Stdout); err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
