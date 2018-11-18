docker rm $(docker ps -aq)
docker rmi `docker images -f "dangling=true" -q`