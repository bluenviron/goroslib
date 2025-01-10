define DOCKERFILE_MSGS
FROM $(BASE_IMAGE)
RUN apk add --no-cache make git
RUN go install mvdan.cc/gofumpt@v0.5.0
WORKDIR /s
COPY go.mod go.sum ./
RUN go mod download
endef
export DOCKERFILE_MSGS

msgs:
	echo "$$DOCKERFILE_MSGS" | docker build -q . -f - -t temp
	docker run --rm -it \
	-v $(shell pwd):/s \
	temp \
	sh -c "make msgs-nodocker"

msgs-nodocker:
	go run ./cmd/msgs-gen
	find ./pkg/msgs -type f -name '*.go' | xargs gofumpt -l -w
