
.PHONY: $(shell ls)

BASE_IMAGE = amd64/golang:1.13-alpine3.10

help:
	@echo "usage: make [action]"
	@echo ""
	@echo "available actions:"
	@echo ""
	@echo "  mod-tidy      run go mod tidy"
	@echo "  format        format source files"
	@echo "  test          run available tests"
	@echo "  msgs          generate messages"
	@echo ""

blank :=
define NL

$(blank)
endef

mod-tidy:
	docker run --rm -it -v $(PWD):/s $(BASE_IMAGE) \
	sh -c "apk add git && cd /s && go get && go mod tidy"

format:
	docker run --rm -it -v $(PWD):/s $(BASE_IMAGE) \
	sh -c "cd /s && find . -type f -name '*.go' | xargs gofmt -l -w -s"

define DOCKERFILE_TEST
FROM $(BASE_IMAGE)
RUN apk add --no-cache make docker-cli git
WORKDIR /s
COPY go.mod go.sum ./
RUN go mod download
COPY . ./
endef
export DOCKERFILE_TEST

test:
	echo "$$DOCKERFILE_TEST" | docker build -q . -f - -t temp
	docker run --rm -it \
	-v /var/run/docker.sock:/var/run/docker.sock:ro \
	temp \
	make test-nodocker

IMAGES = $(shell echo test-images/*/ | xargs -n1 basename)

test-nodocker:
	$(foreach IMG,$(IMAGES),docker build -q test-images/$(IMG) -t goroslib-test-$(IMG)$(NL))
	$(eval export CGO_ENABLED = 0)
	go test -v ./msg-utils
	go test -v ./tcpros
	go test -v ./xmlrpc
	go test -v .
	go test -v ./msgs/...
	$(foreach f,$(shell ls example/*),go build -o /dev/null $(f)$(NL))

define DOCKERFILE_MSGS
FROM $(BASE_IMAGE)
RUN apk add --no-cache make docker-cli git
WORKDIR /s
COPY go.mod go.sum ./
RUN go mod download
endef
export DOCKERFILE_MSGS

msgs:
	echo "$$DOCKERFILE_TEST" | docker build -q . -f - -t temp
	docker run --rm -it \
	-v $(PWD):/s \
	temp \
	make msgs-nodocker

msgs-nodocker:
	cd /s && go run ./msg-gen/main.go
	cd /s && find ./msgs -type f -name '*.go' | xargs gofmt -l -w -s
