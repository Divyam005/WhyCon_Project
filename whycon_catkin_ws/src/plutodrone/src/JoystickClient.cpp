
#include <plutodrone/JoystickClient.h>

bool armState=false;
int localizationSate=0;

void JoystickClient::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{




for(int i=0;i<msg->axes.size();i++)
ROS_INFO_STREAM("Axes: "<< msg->axes[i]);


rcJoystick[0]=1500+(-(msg->axes[2])*180);
rcJoystick[1]=1500+(msg->axes[3]*180);

rcJoystick[3]=1500+(-(msg->axes[0])*500);
rcJoystick[2]=1500+(msg->axes[1]*500);

if(msg->buttons[5]==1)
{
if(!armState)
{rcJoystick[7]=1500;
  armState=true;
}
else
{rcJoystick[7]=1200;
  armState=false;
}
}
if(msg->buttons[4]==1)
{
if(!localizationSate)
{

rcJoystick[5]=1850;

localizationSate=1;
}
else
{

  rcJoystick[5]=1250;
  localizationSate=0;
}
}

// if(!localizationSate)
// {rcJoystick[0]=1500+(-(msg->axes[3])*500);
// rcJoystick[1]=1500+(msg->axes[4]*500);
// }

printf("localization state= %i\n",localizationSate );

//for(int i=0;i<msg->buttons.size();i++)
//ROS_INFO_STREAM("Buttons: "<< msg->buttons[i]);


}


int* JoystickClient::getRcJoystick()
{


return rcJoystick;


}
