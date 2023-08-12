// main package.
package main

import (
	"fmt"
	"os"

	"github.com/alecthomas/kong"

	"github.com/bluenviron/goroslib/v2/pkg/conversion"
)

var cli struct {
	Path string `arg:"" help:"path pointing to a ROS package"`
}

func run(args []string) error {
	parser, err := kong.New(&cli,
		kong.Description("Convert all messages, services and actions definions in a ROS package into Go structs."),
		kong.UsageOnError())
	if err != nil {
		return err
	}

	_, err = parser.Parse(args)
	if err != nil {
		return err
	}

	return conversion.ImportPackageRecursive(cli.Path)
}

func main() {
	if err := run(os.Args[1:]); err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
