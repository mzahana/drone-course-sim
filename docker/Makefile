# Thin wrapper over ../build.sh, which picks the CPU or GPU image for this
# machine. Use the script directly; these exist for muscle memory.
# `make PX4_VERSION=v1.16.0 build` and `make TORCH_VARIANT=cu126 build-gpu` reach the script.
export PX4_VERSION TORCH_VARIANT

.PHONY: build build-cpu build-gpu rebuild shell
build:
	cd .. && ./build.sh

build-cpu:
	cd .. && ./build.sh --cpu

build-gpu:
	cd .. && ./build.sh --gpu

rebuild:
	cd .. && ./build.sh --no-cache

shell:
	docker run -it --rm drone-course-sim:jazzy bash
