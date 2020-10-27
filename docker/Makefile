build:
	docker build -t vins_mono_docker .

run:
	docker run -e DISPLAY=$DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /data:/data \
		--network="host" \
		-it \
		--rm vins_mono_docker

join_container:
	docker exec -it vins_mono_docker bash
