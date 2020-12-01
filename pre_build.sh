if test ! -f "/root/catkin_ws/src/pwc_net_ros/model/pwc_net.caffemodel"; then
ln -s /root/PWC-Net/Caffe/model/pwc_net.caffemodel /root/catkin_ws/src/pwc_net_ros/model/pwc_net.caffemodel
fi

patch -o /root/catkin_ws/src/pwc_net_ros/model/pwc_net_test.prototxt /root/PWC-Net/Caffe/model/pwc_net_test.prototxt /root/catkin_ws/src/pwc_net_ros/model/pwc_net_test.diff

