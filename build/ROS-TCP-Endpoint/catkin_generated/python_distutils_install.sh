#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/duefor/climbrobot_mjk/src/ROS-TCP-Endpoint"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/duefor/climbrobot_mjk/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/duefor/climbrobot_mjk/install/lib/python3/dist-packages:/home/duefor/climbrobot_mjk/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/duefor/climbrobot_mjk/build" \
    "/usr/bin/python3" \
    "/home/duefor/climbrobot_mjk/src/ROS-TCP-Endpoint/setup.py" \
    egg_info --egg-base /home/duefor/climbrobot_mjk/build/ROS-TCP-Endpoint \
    build --build-base "/home/duefor/climbrobot_mjk/build/ROS-TCP-Endpoint" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/duefor/climbrobot_mjk/install" --install-scripts="/home/duefor/climbrobot_mjk/install/bin"
