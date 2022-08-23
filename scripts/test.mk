test-cmd:
	go build -o /dev/null ./cmd/...

test-examples:
	go build -o /dev/null ./examples/...

test-pkg:
	go test -v -race -coverprofile=coverage-pkg.txt ./pkg/...

test-root:
	$(foreach IMG,$(shell echo testimages/*/ | xargs -n1 basename), \
	docker build -q testimages/$(IMG) -t goroslib-test-$(IMG)$(NL))
	go test -v -race -coverprofile=coverage-root.txt .

test-nodocker: test-cmd test-examples test-pkg test-root

define DOCKERFILE_TEST
FROM $(BASE_IMAGE)
RUN apk add --no-cache make docker-cli git gcc musl-dev
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
