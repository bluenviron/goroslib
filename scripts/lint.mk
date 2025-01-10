lint:
	docker run --rm -v $(shell pwd):/app -w /app \
	$(LINT_IMAGE) \
	golangci-lint run -v
