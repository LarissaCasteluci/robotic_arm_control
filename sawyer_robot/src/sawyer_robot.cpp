
int position=0;
#include "dependencies.hpp"
#include "sawyer_comp.hpp"

int sleepTime = 10000;
// Global variables (also modified by the topic subscriber):
struct timeval tv;	//timevalue
  unsigned int currentTime_updatedByTopicSubscriber=0;
  float simulationTime=0.0;

// Topic subscriber callbacks:
void simulationTimeCallback(const std_msgs::Float32& simTime)
{
    simulationTime=simTime.data; //update data
}

//This is already defined in cv_bridge, anyway, just to know
  /*namespace cv_bridge
  {
    //Class has header, encoding type and the image itself
    class CvImage
    {
      public:
        std_msgs::Header header;
        std::string encoding;
        cv::Mat image;
    };

    typedef boost::shared_ptr<CvImage> CvImagePtr; ------->nome do trampo é template
    typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

  } */

// Main code:
int main(int argc,char* argv[])
{
    // The robot motor velocities and the sensor topic names are given in the argument list
    // (when V-REP launches this executable, V-REP will also provide the argument list)

	//TOPICS

    std::string headCameraTopic;
    std::string wristCameraTopic;
    std::string simulationTimeTopic;
    std::string ikTopic;
    std::string jointsTopic;

    if (argc>=5) //Declaracao dos nomes dos Tópicos
    {

	        headCameraTopic=argv[1];
	        wristCameraTopic=argv[2];
          simulationTimeTopic=argv[3];
          ikTopic=argv[4];
          jointsTopic=argv[5];
	      headCameraTopic="/"+headCameraTopic;
	      wristCameraTopic="/"+wristCameraTopic;
        simulationTimeTopic="/"+simulationTimeTopic;
        ikTopic="/"+ikTopic;
        jointsTopic="/"+jointsTopic;
    }
    else //senão erro
    {
        printf("Sawyer here, indicate following arguments: 'ALL TOPICS'!\n");
        sleep(5000);
        return 0;
    }

    // Create a ROS node. The name has a random component:
      int _argc = 0;
      char** _argv = NULL;
      if (gettimeofday(&tv,NULL)==0)
          currentTime_updatedByTopicSubscriber=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
      std::string nodeName("sawyer_robot");
      std::string randId(boost::lexical_cast<std::string>(currentTime_updatedByTopicSubscriber+int(999999.0f*(rand()/(float)RAND_MAX))));
      nodeName+=randId;


    ros::init(_argc,_argv,"sawyer_robot");
    if(!ros::master::check()) return(0);

    ros::NodeHandle n;
    printf("Sawyer just started with node name %s\n",nodeName.c_str());

    // 1. Let's subscribe to the sensor and simulation time stream
    ros::Subscriber subSimulationTime=n.subscribe(simulationTimeTopic.c_str(),1,simulationTimeCallback);
    headCamera hC = headCamera(headCameraTopic);
    wristCamera wC = wristCamera(wristCameraTopic);

    // 2. Let's prepare publishers for the joints positions:
    //Declarao dos Publishers das Juntas
      ros::Publisher ikPub=n.advertise<std_msgs::Float32>(ikTopic.c_str(),1);
      ros::Publisher jointsPub=n.advertise<std_msgs::Float32MultiArray>(jointsTopic.c_str(),1);

      ros::Publisher sawyer_omnipadPub = n.advertise<std_msgs::Float32>("sawyer_omnipadTopic",1);

    // 3. Finally we have the control loop:
    float driveBackStartTime=-99.0f;
    unsigned int currentTime;
    if (gettimeofday(&tv,NULL)==0)
    {
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
        currentTime=currentTime_updatedByTopicSubscriber;
    }

   //Declaração das Variáveis das Posições Desejadas das Juntas
      float dj1P; //desired joint 1 position
        float dj2P;
        float dj3P;
        float dj4P;
        float dj5P;
        float dj6P;
        float dj7P;
        float ik; //ik solver

        std_msgs::Float32MultiArray j;
        std_msgs::Float32 d;

        float command; //command to Omnipad

        float d2r = 3.141596/180;

    while (ros::ok())
    {
	     // this is the control loop (very simple, just as an example)
        if (gettimeofday(&tv,NULL)==0)
        {
            currentTime=tv.tv_sec;
            if (currentTime-currentTime_updatedByTopicSubscriber>50)
                break; // we didn't receive any sensor information for quite a while... we leave
        }
         // Printing Simulation Time
	      //printf("Simulation time is %f \n", simulationTime);

        //Posições Desejadas
        if(position==0){

          dj1P =0*d2r;
            dj2P =0*d2r - 90*d2r; //posição inicial é -90
            dj3P =0*d2r;
            dj4P =0*d2r;
            dj5P =0*d2r;
            dj6P =0*d2r;
            dj7P =0*d2r;
            command=1;
            ik=0;
          }
          else if(position==1)
          {
              ik=1;
              command=0;
              wC.wristAnalsys();
          }

        //Publicação dos parametros
          d.data=command;
          sawyer_omnipadPub.publish(d);

          d.data=ik;
          ikPub.publish(d);

          //now contents
          j.data={ik,dj1P,dj2P,dj3P,dj4P,dj5P,dj6P,dj7P};
          jointsPub.publish(j);

        // handle ROS messages:
        ros::spinOnce();

        // sleep a bit:
        usleep(sleepTime);
    }
      ros::shutdown();
      printf("sawyer is over!\n");
    return(0);
}
