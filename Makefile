BASE_IMAGE = golang:1.18-alpine3.15
LINT_IMAGE = golangci/golangci-lint:v1.49.0

.PHONY: $(shell ls)

help:
	@echo "usage: make [action]"
	@echo ""
	@echo "available actions:"
	@echo ""
	@echo "  mod-tidy      run go mod tidy"
	@echo "  format        format source files"
	@echo "  test          run tests"
	@echo "  lint          run linter"
	@echo "  msgs          generate standard messages"
	@echo ""

blank :=
define NL

$(blank)
endef

include scripts/*.mk
