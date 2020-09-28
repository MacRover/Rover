DISPLAY=`grep -oP "(?<=nameserver ).+" /etc/resolv.conf`:0.0
LIBGL_ALWAYS_INDIRECT=1

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    macrover/rover:wsl