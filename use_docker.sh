#!/bin/bash

#options for script:
# --help: Display help
# --build: Build the docker image
# --run: Run the docker container
# --stop: Stop the docker container
# --enter: Enter the docker container with shell

display_help() {
    echo "Usage: ./use_docker.sh [OPTION]"
    echo "Options:"
    echo "  --help: Display help"
    echo "  --build: Build the docker image"
    echo "  --run: Run the docker container"
    echo "  --stop: Stop the docker container"
    echo "  --enter: Enter the docker container with shell"
}

# if --help is passed, display help
if [ "$1" == "--help" ]; then
    display_help
    exit 0
fi

# if --build is passed, build the docker image
if [ "$1" == "--build" ]; then
    docker build -t ur5e_unity:latest .
    exit 0
fi

# if --run is passed, run the docker container
if [ "$1" == "--run" ]; then
    xhost +local:root
    docker run -t -d --privileged --net host --name ur5e_unity --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" ur5e_unity:latest
    exit 0
fi

# if --stop is passed, stop the docker container
if [ "$1" == "--stop" ]; then
    xhost -local:root
    docker stop ur5e_unity
    docker rm ur5e_unity
    exit 0
fi

# if --enter is passed, enter the docker container with shell
if [ "$1" == "--enter" ]; then
    docker exec -it ur5e_unity /bin/bash
    exit 0
fi

# if no option is passed, display help
display_help
exit 1