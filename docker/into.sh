#!/bin/bash
docker exec --user "docker_gmmloc" -it gmmloc \
        /bin/bash -c ". /ros_entrypoint.sh; cd /home/docker_gmmloc; /bin/bash"