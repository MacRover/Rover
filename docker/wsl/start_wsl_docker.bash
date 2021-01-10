# modify & uncomment this line to automatically start X11
# cmd.exe /C start "C:\<WINDOWS_STYLE_FILE_PATH>"

DISPLAY=`grep -oP "(?<=nameserver ).+" /etc/resolv.conf`:0.0
LIBGL_ALWAYS_INDIRECT=1

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    macrover/rover:wsl