sudo docker run -it --net=host --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY \
    -v `pwd`/share:/root/share \
    -w /root/share \
    --name ros2_jazzy2 \
    lhs223/jazzy_container:latest \
    bash
