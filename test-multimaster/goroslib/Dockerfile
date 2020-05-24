FROM golang:1.14

WORKDIR /s

COPY go.mod go.sum ./
RUN go mod download

COPY . ./

RUN go build -o /out \
    ./test-multimaster/goroslib/main.go

ENTRYPOINT [ "/out" ]
