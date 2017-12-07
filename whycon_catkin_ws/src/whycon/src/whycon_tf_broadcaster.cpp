/*
Author Name: Sidhartha Sankar Prusti
Email: sidharthaprusti.84@gmail.com

File Name: whycon_tf_broadcaster.cpp
Theme: eYSIP-2017

Created: 25th May 2017

Prupose: File to store the pose data of Whycons detected and then publish the tf between
         whycons and camera.
*/

//Libraries
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

namespace whycon_mapping
{
  class WhyconMapping
  {
  public:

    //WhyconInfo: to store the pose data of the Whycons
    struct WhyconInfo
    {
      geometry_msgs::Pose pose_to_world;              // Position with respect to world's origin
      tf::Transform tf_to_world;                      // TF with respect to world
      std::vector<geometry_msgs::Pose> pose_to_prev;  // pose of whycon with respect to its previous whycon
    };

    /**********************************
    Function name : WhyconMapping
    Functionality : Constructor for the WhyconMapping class
    Arguments     : pointer to the ROS node
    Return Value  : None
    Example Call  : Called on declaration of class variable
    ***********************************/
    WhyconMapping(ros::NodeHandle *nh) :num_of_whycons_ (1)                   // Number of whycons to be detected
    {
      //check for the valid number of whycon as the parameter
      if(!nh->getParam("/whycon_mapping/num_of_whycons", num_of_whycons_)) 
        throw std::runtime_error("Private parameter \"num_of_whycons\" is missing");
    
      //Resize whycon container
      whycon_.resize(num_of_whycons_);
      whycon_0_tf_others.resize(num_of_whycons_-1);

      //subsequently reduce the number of child frame to parent by 1
      for (int i = 0; i < num_of_whycons_; ++i)
        whycon_[i].pose_to_prev.resize(num_of_whycons_-i-1);
    }

    /**********************************
    Function name : WhyconPoseCallback
    Functionality : Call back function for the Whycon pose subscriber
    Arguments     : poses_msg of geometry_msgs/PoseArray type
    Return Value  : None
    Example Call  : Called on Whycon pose subscription
    ***********************************/
    void WhyconPoseCallback(const geometry_msgs::PoseArrayConstPtr& poses_msg)
    { 
      // Set initial status of the center detection as flase
      center_detected=false;
      tf::Quaternion q(0.0,0.0,0.0,1);
    
      //Store the received pose data as the tf data between World frame and Whycon_n
      for(int i=0;i<num_of_whycons_;i++)
      {
        whycon_[i].pose_to_world=poses_msg->poses[i];
        whycon_[i].tf_to_world.setOrigin(tf::Vector3(poses_msg->poses[i].position.x,poses_msg->poses[i].position.y, poses_msg->poses[i].position.z));
        whycon_[i].tf_to_world.setRotation(q);
      }
  
      now=ros::Time::now();
      tfBetweenWhycon();
      publishTfs();
     }

  private :

    /**********************************
    Function name : tfBetweenWhycon
    Functionality : Compute the transform data between world and whycon
                                transform data between each one of Whycons
    Arguments     : None
    Return Value  : None
    Example Call  : tfBetweenWhycon()
    ***********************************/
    void tfBetweenWhycon(void)
    {
      // loop to look up and store whycon tf data
      for (int i = 0; i < num_of_whycons_; ++i)
      {
        std::stringstream whycon_tf_parent;
        whycon_tf_parent<<"whycon_c_"<<i;
        for (int k=i+1; k<num_of_whycons_; ++k)
        {
          // Inner loop to look up for transform between parent whycon_i and child whycon_k
          //i = outer loop index
          //k = inner loop index

          std::stringstream whycon_tf_from;
          std::stringstream whycon_tf_child;
          whycon_tf_from<<"whycon_"<<k;
          whycon_tf_child<<"whycon_c_"<<k;
          whycon_br.sendTransform(tf::StampedTransform(whycon_[i].tf_to_world,now,"world",whycon_tf_parent.str()));
          whycon_br.sendTransform(tf::StampedTransform(whycon_[k].tf_to_world,now,"world",whycon_tf_from.str()));
          try 
          {
            whycon_lt.waitForTransform(whycon_tf_parent.str(), whycon_tf_from.str(), ros::Time(0), ros::Duration(1.0) );
            whycon_lt.lookupTransform(whycon_tf_parent.str(), whycon_tf_from.str(), ros::Time(0), transform_tmp);
          } 
          catch (tf::TransformException ex) 
          {
            ROS_ERROR("%s",ex.what());
          }

          if (i==0)
          {
            whycon_0_tf_others[k-1]=transform_tmp;
            whycon_br.sendTransform(tf::StampedTransform(whycon_0_tf_others[k-1],ros::Time::now(),whycon_tf_parent.str(),whycon_tf_child.str()));
          }

          /* Store the datas of tf between current Whycon and previous deteced whycon*/
          whycon_[i].pose_to_prev[k-i-1].position.x=transform_tmp.getOrigin().x();
          whycon_[i].pose_to_prev[k-i-1].position.y=transform_tmp.getOrigin().y();
          whycon_[i].pose_to_prev[k-i-1].position.z=transform_tmp.getOrigin().z();

          whycon_[i].pose_to_prev[k-i-1].orientation.x=transform_tmp.getRotation().x();
          whycon_[i].pose_to_prev[k-i-1].orientation.y=transform_tmp.getRotation().y();
          whycon_[i].pose_to_prev[k-i-1].orientation.z=transform_tmp.getRotation().z();
          whycon_[i].pose_to_prev[k-i-1].orientation.w=transform_tmp.getRotation().w();
        }
      }
    }

    /**********************************
    Function name : publishTfs
    Functionality : publish transform between world and whycon
                            transform between each one of Whycons
                    //brief Function to publish all known TFs
    Arguments     : None
    Return Value  : None
    Example Call  : publishTfs()
    ***********************************/
    void publishTfs(void)
    {
      for(int i = 0; i < num_of_whycons_; i++)
      {
        // Actual Whycon
        std::stringstream whycon_tf_id;
        whycon_tf_id << "whycon_" << i;
        //publish transform between world and Whycon_n
        //where n--> number of whycon detected
        whycon_br.sendTransform(tf::StampedTransform(whycon_[i].tf_to_world,ros::Time::now(),"world",whycon_tf_id.str()));
      }

      //set Camera frame as the World frame and Publish the same
      transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      whycon_br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world", "camera"));
    }

    //brief Container holding WhyconInfo data about all detected whycons
    std::vector<WhyconInfo> whycon_;

    //time stamped transform data between Whycon numbered as 0 and others
    std::vector<tf::StampedTransform> whycon_0_tf_others;

    //store current ros Time 
    ros::Time now;

    //transform broadcaster
    tf::TransformBroadcaster whycon_br;

    //transform listener
    tf::TransformListener whycon_lt;

    //time stamped transform data
    tf::StampedTransform transform_tmp;

    //store the transform data
    tf::Transform transform;
    //Launch file params
    //To store the number of Whycons to be detected
    int num_of_whycons_;

    //store the status of the centre detetcion
    bool center_detected;
  }; //end of WhyconMapping class
}  //end of whycon_mapping namespace

/**********************************
Function name : main
Functionality : main function for the node "whycon_mapping"
Arguments     : argc, argv
Return Value  : None
Example Call  : called on node initialisation
***********************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "whycon_mapping");
  ros::NodeHandle nh;

  // whycon mapping object
  whycon_mapping::WhyconMapping obj(&nh);
  
  //ROS subscriber
  ros::Subscriber sub = nh.subscribe("whycon/poses", 10, &whycon_mapping::WhyconMapping::WhyconPoseCallback,&obj);
  ros::spin();
  return(EXIT_SUCCESS);
};