
#include <plutodrone/Position.h>
#include <plutodrone/Protocol.h>


timeval tv;
double lasttime, nowtime;
double deltaT=0;
bool iffirstdata=true;
int posArray[4];
Protocol pro1;

void Position::positionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{




  poseX=msg->poses[0].position.x;
  poseY=msg->poses[0].position.y;
  poseZ=msg->poses[0].position.z;

  ROS_INFO_STREAM("I heard: X [ ]"<< poseX);
  ROS_INFO_STREAM("I heard: Y [ ]"<< poseY);

  if(iffirstdata)
  {



   gettimeofday (&tv, NULL);

   lastPostionTime=tv.tv_usec;

   iffirstdata=false;

  }

  else

  {
   gettimeofday (&tv, NULL);

   nowtime=tv.tv_usec;

   deltaT=(nowtime-lastPostionTime)*0.001;

   lastPostionTime=nowtime;

   ROS_ERROR_STREAM("Delta T ="<< deltaT);


  }

    posArray[0]=(int)(poseX*100);
    posArray[1]=(int)(poseY*100);
    posArray[2]=(int)(poseZ*100);
    posArray[3]=(int)(deltaT);

    pro1.sendRequestMSP_SET_POS(posArray);



}




void Position::setPoseX(float x)
{


poseX=x;


}



void Position::setPoseY(float y)
{


poseY=y;


}

void Position::setPoseZ(float z)
{


poseZ=z;


}




float Position::getPoseX()
{



return poseX;


}


float Position::getPoseY()
{



return poseY;


}

float Position::getPoseZ()
{



return poseZ;


}


bool Position::isPositionTimeOut()
{

if(((tv.tv_usec-lastPostionTime)*0.001)>500)
return true;

else
return false;



}
