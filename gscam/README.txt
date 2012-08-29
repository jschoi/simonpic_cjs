See http://code.google.com/p/brown-ros-pkg/wiki/gscam for documentation.

ipcamera:
export GSCAM_CONFIG="gst-launch souphttpsrc location=http://[user]:[password]@[camera_ip]/mjpg/video.mjpg ! jpegdec ! v4l2sink device=/dev/video0"
