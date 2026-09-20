IMAGE ?= drone-course-sim:jazzy
PX4_VERSION ?= v1.17.0

.PHONY: build rebuild shell
build:
	docker build -t $(IMAGE) --build-arg PX4_VERSION=$(PX4_VERSION) -f docker/Dockerfile ..

rebuild:
	docker build --no-cache -t $(IMAGE) --build-arg PX4_VERSION=$(PX4_VERSION) -f docker/Dockerfile ..

shell:
	docker run -it --rm $(IMAGE) bash
