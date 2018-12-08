docker run --rm --privileged multiarch/qemu-user-static:register
docker build -t jetson_cross -f Dockerfile .