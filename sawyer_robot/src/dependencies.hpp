
#include <stdio.h>
  #include <stdlib.h>
  #include <ros/ros.h>
  #include <image_transport/image_transport.h>
  #include <sensor_msgs/image_encodings.h> // inclui os tipos decofificação de imagem
  #include <opencv2/core/core.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <cv_bridge/cv_bridge.h>
  #include "std_msgs/Float32.h"
  #include "std_msgs/Float32MultiArray.h"
  #include "sensor_msgs/Image.h"
  #include <sstream> // for converting the command line parameter to integer
  #include <cmath>
  //#include <cstring>
