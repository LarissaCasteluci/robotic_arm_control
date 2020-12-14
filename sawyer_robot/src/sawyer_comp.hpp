#ifndef HEADCAMERA
#define HEADCAMERA

static const std::string OPENCV_WINDOW = "Sawyer_head";

class headCamera
{
  /*Class
    Caracteristics:
    n, it and imageSub
    Functions:
    headCamera, ~headCamera, //constructor and destructor
    imageCb*/
  //Declare caracteristics of an object: n,it and imageSub
  ros::NodeHandle n; //Name node
  image_transport::ImageTransport it; //Naming image_transport
  image_transport::Subscriber imageSub; //Naming image Subscriber
  image_transport::Publisher imagePub; //Publisher to CV
  //variables
  cv_bridge::CvImagePtr cv_ptr; //create pointer
  cv::Mat intermediate_mat;
  std::vector<cv::Point2f> centerContours;
  std::vector<cv::Point2f> centerContours_filtered;
  int _switch=0;
  int counter_head =0;
  int counter_prior=0;


  public:
    headCamera(std::string headCameraTopic) //declare function headCamera
    :it(n)
    {
      imageSub = it.subscribe(headCameraTopic.c_str(), 1, &headCamera::imageCb, this);

    }

    ~headCamera()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    //Função para lidar com erros, try e catch
    void imageCb(const sensor_msgs::ImageConstPtr& msg)   ///ImageConstPtr= cnt pointer to Image
    {

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //copy image so we can mess with it
        intermediate_mat = findImageContours(cv_ptr->image, centerContours);
        FindCenter(centerContours,cv_ptr->image.cols,centerContours_filtered,counter_head,counter_prior,_switch);
        omnipadControl(centerContours_filtered,_switch);
        //intermediate_mat = cv_ptr->image;
        //cv::imwrite("../../Sawyer_head/headCamera"+ boost::lexical_cast<std::string>(simulationTime)+".jpg", cv_ptr->image);
        cv::namedWindow(OPENCV_WINDOW);
        cv::imshow(OPENCV_WINDOW, intermediate_mat);
        cv::waitKey(10);
        counter_head ++;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    }

    void omnipadControl(std::vector<cv::Point2f> centerContours_filtered,int &_switch)
    {
      if(_switch==1)
      {
        position=1;
        _switch=0;

      }
    }

    void FindCenter(std::vector<cv::Point2f> centerContours, float cols, std::vector<cv::Point2f> &centerContours_filtered,int counter_head, int &counter_prior, int &_switch)
    {

      for(int i=0;i<centerContours.size();i++)
      {
          if (centerContours[i].x >cols/2-5 & centerContours[i].x < cols/2 + 5)
          {
            if(centerContours_filtered.size()==0)
            {
              _switch=1;
              centerContours_filtered.push_back(centerContours[i]);
              if (position==0) counter_prior=counter_head;
            }
            else if(counter_head-counter_prior>10)
            {
              _switch=1;
              centerContours_filtered.push_back(centerContours[i]);
              counter_prior=counter_head;
            }
          }
      }
    }

    cv::Mat findImageContours(cv::Mat intermediate_mat, std::vector<cv::Point2f> &centerContours)
    {
      // find grayscale, Blur the image and set up the detector with default parameters.
        cv::Mat blurred;
        cv::Mat hsv_channels[3];
        cv::split(intermediate_mat, hsv_channels ); //obtain gray_scale from HSV
        cv::blur(hsv_channels[2], blurred, cv::Size(3,3));

      // Apply threshold and find contours
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::Mat threshold_output;
      /// cv::threshold(input, output, threshold value, maximum value to use with the THRESH_BINARY, thresholding type);
      cv::threshold(hsv_channels[2], threshold_output, 100, 255, cv::THRESH_BINARY );
      /// findContours( input, output, Optional output vector, containing information about the image topology
      //mode:all contours,CV_CHAIN_APPROX_SIMPLE,  Optional offset by which every contour point is shifted );
      cv::findContours( threshold_output, contours, hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );


      //filtering a bit
      std::vector<std::vector<cv::Point>> contours_filtered;
      for( int i = 0; i < contours.size(); i++ )
         {
           float Area=contourArea(contours[i]);
           if(Area<1000&Area>10)
           {
            contours_filtered.push_back(contours[i]);
           }
         }

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contours_filtered.size() );
      std::vector<cv::Rect> boundRect(contours_filtered.size());
      std::vector<cv::Point2f> center(contours_filtered.size());
      std::vector<float> radius( contours_filtered.size() );


      for( int i = 0; i < contours_filtered.size(); i++ )
         {
           //Apply contours to polygons
           ///approxPolyDP(input,output,Parameter specifying the approximation accuracy,If true, the approximated curve is closed )
           cv::approxPolyDP( cv::Mat(contours_filtered[i]), contours_poly[i], 3, true );
           //calculates the up-right bounding rectangle of a point set.
           boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
           //Finds a circle of the minimum area enclosing a 2D point set.
           cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
         }

         /// Draw polygonal contour + bonding rects + circles
         cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
         for( int i = 0; i< contours_filtered.size(); i++ )
            {
              //Scalar is simply a convenient container for 1, 2, 3 or 4 floating point values
              cv::Scalar color = cv::Scalar( 255, 255, 255 );
              //drawContours(image, contours, contour to draw, color, thickness, lineType, hierarchy,maxLevel,Point offset)
              cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
              //draw a rectangle
              //(input, vertex 1,vertex opposite to 1, color,thickness,lineType,shift)
            //  cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
              //draw a circle
              cv::circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
            }
            centerContours=center;
            return drawing;
    }

    cv::Mat thresholdRed(cv_bridge::CvImagePtr cv_ptr)
    {
      cv::Mat hsv_image;
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);//----------------------------->AQUI
      cv::Mat red_hue_image;
      cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_hue_image);

      return red_hue_image;
    }

    cv::Mat geometricTransformation(cv_bridge::CvImagePtr cv_ptr)
    {
      int rows = cv_ptr->image.rows;
      int cols= cv_ptr->image.cols;
      cv::Point2f center =cv::Point2f(rows/2,cols/2);
      cv::Mat rotation= cv::getRotationMatrix2D(center,180,1); //centro, rotação, scaling
      cv::Mat teste(rows, cols,CV_8UC(3)); //Cria uma matriz de 3 canais de 8bits
      cv::Size dsize=cv::Size(rows,cols);
      cv::warpAffine(cv_ptr->image,teste, rotation,dsize); //entrada, saída, matriz de transformação, tamanho
      cv::flip(teste,cv_ptr->image,1);
      return cv_ptr->image;
    }


};

#endif

#ifndef WRISTCAMERA
#define WRISTCAMERA

class wristCamera
{
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber imageSub;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat intermediate_mat;

  int counter_wrist=0;
  int counter_analysis=0;

  public:
    wristCamera(std::string wristCameraTopic)
    :it(n)
    {
      imageSub = it.subscribe(wristCameraTopic.c_str(), 1, &wristCamera::imageCb, this);
    }

    ~wristCamera()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        counter_wrist++;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }

    void wristAnalsys()
    {
      counter_analysis++;
    /*  cv::namedWindow("Sawyer_wrist");
      intermediate_mat=thresholdRed(cv_ptr);
      cv::imshow("Sawyer_wrist", intermediate_mat);
      cv::waitKey(10); */

  //    if(counter_analysis>200)
  //    {
    //    counter_analysis=0;
    //    position=0;
  //    }
    }

};

#endif
