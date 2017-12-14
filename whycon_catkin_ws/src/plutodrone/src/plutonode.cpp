  #include "ros/ros.h"
  #include "std_msgs/String.h"
  #include "plutodrone/PlutoPilot.h"
  #include <geometry_msgs/PoseArray.h>
  #include <sys/time.h>
  #include <boost/thread.hpp>
  #include <boost/chrono.hpp>
  #include <pthread.h>
  #include <unistd.h>
  #include <plutodrone/Common.h>
  #include <plutodrone/Protocol.h>
  #include <plutodrone/JoystickClient.h>
  #include <plutodrone/Position.h>






  #define PORT 23



  using namespace std;

  timeval tv;
  double lasttime, nowtime;
  double deltaT;
  bool iffirstdata=true;

  bool isSocketCreate=false;

   Communication com;
   Protocol pro;
   JoystickClient joystick;
   Position position;


  ros::ServiceClient serviceClient;
  plutodrone::PlutoPilot service;

  //float kx=90,  ky=-90;
  float kx=70,  ky=-70;
  //float kx=100,  ky=-100;

  //float kd_x=0, kd_y=0;

  float kd_x=-40000, kd_y=40000;

  float desiredX=0, desiredY=0;

  float velocityX=0, velocityY=0;

  float lastX=0,currentX=0, lastY=0, currentY=0;;


  int PID_x, PID_y;

  int userRC[8]={0,0,0,0,0,0,0,0};


  void limit(int *value)

  {

    if(*value>100)
    *value=100;

    if(*value<-100)
    *value=-100;

  }


  void limitVelocity(float *value)

  {

    if(*value>1)
    *value=1;

    if(*value<-1)
    *value=-1;






  }


 bool isValidRC(int rcvalue)
{


if(rcvalue>999&&rcvalue<2001)
return true;
else
return false;




}


  void *createSocket(void *threadid)
  {


   isSocketCreate=com.connectSock();


   pthread_exit(NULL);
  };



  void *writeFunction(void *threadid)
  {

  std::vector<int> requests;

  requests.push_back(MSP_RC);
//  requests.push_back(MSP_ATTITUDE);
  //requests.push_back(MSP_ANALOG);
  //requests.push_back(MSP_ALTITUDE);
  //requests.push_back(MSP_FC_VERSION);
  //requests.push_back(MSP_ACC_TRIM);
  //requests.push_back(MSP_RAW_IMU);



  //int channels [8]={1500,1500,1500,1500,1500,1250,1500,1200};


  do {



  if(0)
  {

  currentX=position.getPoseX();
  currentY=position.getPoseY();

  if(deltaT>0)
  {
  velocityX=(currentX-lastX)/deltaT;
  velocityY=(currentY-lastY)/deltaT;
  }
  else
  {

    velocityX=0;
    velocityY=0;


  }


  limitVelocity(&velocityX);
  limitVelocity(&velocityY);


  PID_x=(int)((kx*(desiredX-currentX)) + kd_x*velocityX);
  PID_y=(int)((ky*(desiredY-currentY)) + kd_y*velocityY);


  printf("velocityX = %f\n",velocityX*1000);
  printf("velocityY = %f\n",velocityY*1000);


  /*
  printf("velocityX = %f\n",PID_x);
  printf("velocityY = %f\n",PID_y);
  */

  lastX=currentX;
  lastY=currentY;

  limit(&PID_x);
  limit(&PID_y);

  printf("PID X= %i\n",PID_x );
  printf("PID Y= %i\n",PID_y);


  joystick.rcJoystick[0]=1500+PID_x; //roll

  joystick.rcJoystick[1]=1500+PID_y; //pitch





  }

  if(!localizationSate)
  {

  for(int i=0; i<8; i++)
  {

   if(isValidRC(userRC[i]))
   {
    joystick.rcJoystick[i]=userRC[i];

   }


  }

  }


  if(position.isPositionTimeOut())
  joystick.rcJoystick[7]=1200;



//  channels[4]=service.response.rcAUX1;
//  ROS_INFO("Sum: %ld", joystick.rcJoystick[4]);
  //printf("i am here\n");
   pro.sendRequestMSP_SET_RAW_RC(joystick.getRcJoystick());
   pro.sendRequestMSP_GET_DEBUG(requests);

   //printf("i am there\n" );
  usleep(100000);

  } while(1);













   pthread_exit(NULL);


  }

  void *readFunction(void *threadid)

  {


  do {
    /* code */
    //


  com.readFrame();
  //usleep(5);

  } while(1);


   pthread_exit(NULL);



  }




  void *serviceFunction(void *threadid)
  {





  while (ros::ok()) {



  if (serviceClient.call(service))
    {
    //  ROS_INFO("Sum: %ld", (long int)service.response.rcAUX1);

         userRC[0]=service.response.rcRoll;
         userRC[1]=service.response.rcPitch;
         userRC[2]=service.response.rcThrottle;
         userRC[3]=service.response.rcYaw;
         userRC[4]=service.response.rcAUX1;
         userRC[5]=service.response.rcAUX2;
         userRC[6]=service.response.rcAUX3;
         userRC[7]=service.response.rcAUX4;


         service.request.accX=accX;
         service.request.accY=accY;
         service.request.accZ=accZ;
         service.request.gyroX=gyroX;
         service.request.gyroY=gyroY;
         service.request.gyroZ=gyroZ;
         service.request.magX=magX;
         service.request.magY=magY;
         service.request.magZ=magZ;
         service.request.roll=roll;
         service.request.pitch=pitch;
         service.request.yaw=yaw;




    }
    else
    {
    //  ROS_ERROR("error in pluto service");

    }

     usleep(100000);

  }


   pthread_exit(NULL);

  }


  /**
   * This tutorial demonstrates simple receipt of messages over the ROS system.
   */
  void chatterCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    ROS_INFO_STREAM("I heard: [ ]"<< msg->poses[0]);

    if(iffirstdata)
    {



     gettimeofday (&tv, NULL);

     lasttime=tv.tv_usec;

     iffirstdata=false;

    }

    else

    {
  gettimeofday (&tv, NULL);

     nowtime=tv.tv_usec;

     deltaT=(nowtime-lasttime)*0.001;

     lasttime=nowtime;

    // ROS_ERROR_STREAM("Delta T ="<< deltaT);


   }

  }





  int main(int argc, char **argv)
  {
   pthread_t thread, readThread, writeThread, serviceThread;
     int rc;





    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "plutonode");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */

    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub = n.subscribe("/whycon/poses", 1000, &Position::positionCallback,&position);


    ros::Subscriber joySub = n.subscribe("joy", 1000, &JoystickClient::joystickCallback, &joystick);



   //cout << "main() : creating thread, " << i << endl;
        rc = pthread_create(&thread, NULL, createSocket, 	(void *)1);

        if (rc){
           cout << "Error:unable to create communication thread," << rc << endl;
           exit(-1);
        }


  pthread_join( thread, NULL);

  if(isSocketCreate)
  {



  //cout << "main() : creating write thread, " << i << endl;
       rc = pthread_create(&writeThread, NULL, writeFunction, 	(void *)2);

       if (rc){
          cout << "Error:unable to create write thread," << rc << endl;
          exit(-1);
       }







      // cout << "main() : creating read thread, " << i << endl;
            rc = pthread_create(&readThread, NULL, readFunction, 	(void *)3);

            if (rc){
               cout << "Error:unable to read create thread," << rc << endl;
               exit(-1);
            }


  serviceClient = n.serviceClient<plutodrone::PlutoPilot>("PlutoService",true);




        //    cout << "main() : creating service thread, " << i << endl;
                 rc = pthread_create(&serviceThread, NULL, serviceFunction, 	(void *)4);

                 if (rc){
                    cout << "Error:unable to service create thread," << rc << endl;
                    exit(-1);
                 }


  }




   //cout << "main() : exited thread, " << i << endl;
  //boost::thread t(createSocket);
   // t.join();

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();





    return 0;
  }
