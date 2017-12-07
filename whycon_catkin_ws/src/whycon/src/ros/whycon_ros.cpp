/*
Author Name: Sidhartha Sankar Prusti
Email: sidharthaprusti.84@gmail.com

File Name: whycon_ros.cpp
Theme: eYSIP-2017

Created: 25th May 2017

Prupose: File to start Whycon detection.
*/

#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
  transformation_loaded = false;
  similarity.setIdentity();

  if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

  n.param("name", frame_id, std::string("whycon"));
  n.param("world_frame", world_frame_id, std::string("world"));
  n.param("max_attempts", max_attempts, 1);
  n.param("max_refine", max_refine, 1);

  n.getParam("outer_diameter", parameters.outer_diameter);
  n.getParam("inner_diameter", parameters.inner_diameter);
  n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
  n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
  n.getParam("roundness_tolerance", parameters.roundness_tolerance);
  n.getParam("circularity_tolerance", parameters.circularity_tolerance);
  n.getParam("max_size", parameters.max_size);
  n.getParam("min_size", parameters.min_size);
  n.getParam("ratio_tolerance", parameters.ratio_tolerance);
  n.getParam("max_eccentricity", parameters.max_eccentricity);

  /* initialize ros */
  int input_queue_size = 1;
  n.param("input_queue_size", input_queue_size, input_queue_size);

  //Check the topic and Subscribe the published "---/image_rect_color" topic
  cam_sub = it.subscribeCamera("/usb_cam/image_rect_color", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  
  image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
  poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
  context_pub = n.advertise<sensor_msgs::Image>("context", 1);
  projection_pub = n.advertise<whycon::Projection>("projection", 1);

  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_model.fromCameraInfo(info_msg);
  if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");

  const cv::Mat& image = cv_ptr->image;

  if (!system)
    system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), parameters);

  is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);

  /*if the number of Whycon is 5 then configure the arena as*/
  if(system->targets==5)
    initialize_order();
  
  if (is_tracking) {
    publish_results(image_msg->header, cv_ptr);
    should_reset = false;
  }
  else if (image_pub.getNumSubscribers() != 0)
    image_pub.publish(cv_ptr);

  if (context_pub.getNumSubscribers() != 0) {
    cv_bridge::CvImage cv_img_context;
    cv_img_context.encoding = cv_ptr->encoding;
    cv_img_context.header.stamp = cv_ptr->header.stamp;
    system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
    context_pub.publish(cv_img_context.toImageMsg());
  }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  should_reset = true;
  return true;
}

/*Configure arena As    
        whycon_4                              Whycon_3
                                                        
                                                   
                          Whycon_0              
                                                       
                                                        
        Whycon_1                              Whycon_2
*/

/**********************************
Function name : initialize_order
Functionality : Function to sort the order of Whycon detected
Arguments     : None
Return Value  : None
Example Call  : initialize_order()
***********************************/
void whycon::WhyConROS::initialize_order(void)
{
  // first detect the center and save it as 0th Whycon
  detect_center();

  //std::cout<<"\nstart of the loop:\n";
  for (int k = 1; k < system->targets;k++)
  {
    //std::cout<<"\nstart of the inner loop:\n";
    for(int i=1;i<system->targets;i++)
    {
      whycon::CircleDetector::Circle& circle = system->detector.circles[i];
      whycon::CircleDetector::Circle circle_swap=circle;

      whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
      
      //Detect Whycon_1
      if (pose.pos[0]<-offset and pose.pos[1]>offset)
      {
        whycon::CircleDetector::Circle& circle_1=system->detector.circles[1];
        whycon::CircleDetector::Circle circle_swap_1=circle_1;

        circle=circle_swap_1;
        circle_1=circle_swap;
        //print_values_initialize_order(k,i);
        continue;
      }
      
      //Detect Whycon_2
      else if (pose.pos[0]>offset and pose.pos[1]>offset)
      {
        whycon::CircleDetector::Circle& circle_1=system->detector.circles[2];
        whycon::CircleDetector::Circle circle_swap_1=circle_1;

        circle=circle_swap_1;
        circle_1=circle_swap;
        //print_values_initialize_order(k,i);
        continue;
      }
      
      //Detect Whycon_3
      else if (pose.pos[0]>offset and pose.pos[1]<-offset)
      {
        whycon::CircleDetector::Circle& circle_1=system->detector.circles[3];
        whycon::CircleDetector::Circle circle_swap_1=circle_1;

        circle=circle_swap_1;
        circle_1=circle_swap;
        //print_values_initialize_order(k,i);
        continue;
      }
    
      //Detect Whycon_4    
      else if (pose.pos[0]<-offset and pose.pos[1]<-offset)
      {
        whycon::CircleDetector::Circle& circle_1=system->detector.circles[4];
        whycon::CircleDetector::Circle circle_swap_1=circle_1;

        circle=circle_swap_1;
        circle_1=circle_swap;
        //print_values_initialize_order(k,i);
        continue;
      }
    }
  }
}

/**********************************
Function name : detect_center
Functionality : determine the Whycon 0 from the detected Whycons
Arguments     : None
Return Value  : None
Example Call  : detect_center()
***********************************/
void whycon::WhyConROS::detect_center(void)
{
  bool first_time=true;
  float min_x=FLT_MIN_EXP,min_y=FLT_MIN_EXP;
  int i=0;
  //std::cout<<"\nInside loop:\n";
  while(i<system->targets)
  {
    whycon::CircleDetector::Circle& circle = system->detector.circles[i];
    whycon::CircleDetector::Circle circle_swap=circle;

    whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
    if (first_time)
    {
      min_x=std::fabs(pose.pos[0]);
      min_y=std::fabs(pose.pos[1]);
      first_time=false;
      continue;
    }

    //Determine the whycon with minimum values of pose
    if (std::fabs(pose.pos[0])<min_x or std::fabs(pose.pos[1])<min_y)
    {
      min_x=std::fabs(pose.pos[0]);
      min_y=std::fabs(pose.pos[1]);

      whycon::CircleDetector::Circle& circle_1=system->detector.circles[0];
      whycon::CircleDetector::Circle circle_swap_1=circle_1;
      circle=circle_swap_1;
      circle_1=circle_swap;
    }
    //print_values_initialize_order(0,i);
    i++;
  }
  center_detected=true;
}


/**********************************
Function name : print_values_initialize_order
Functionality : print the pose of the dected Whycons
Arguments     : index of the position to be optimized
                index of the position which is not opitimized
Return Value  : None
Example Call  : print_values_initialize_order(0,2);
***********************************/
void whycon::WhyConROS::print_values_initialize_order(int to_be_optimized,int not_optimized)
{
  whycon::CircleDetector::Circle circle_tmp=system->detector.circles[not_optimized];
  whycon::LocalizationSystem::Pose pose_tmp = system->get_pose(circle_tmp);

  std::cout<<"tmp_"<<not_optimized<<":   ";
  std::cout<<"x: "<<pose_tmp.pos[0]<<"\t";
  std::cout<<"y: "<<pose_tmp.pos[1]<<"\n";

  whycon::CircleDetector::Circle circle_tmp_1=system->detector.circles[to_be_optimized];
  whycon::LocalizationSystem::Pose pose_tmp_1 = system->get_pose(circle_tmp_1);

  std::cout<<"tmp_"<<to_be_optimized<<":   ";
  std::cout<<"x: "<<pose_tmp_1.pos[0]<<"\t";
  std::cout<<"y: "<<pose_tmp_1.pos[1]<<"\n";
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
  bool publish_images = (image_pub.getNumSubscribers() != 0);
  bool publish_poses = (poses_pub.getNumSubscribers() != 0);
  
  if (!publish_images && !publish_poses) return;
  
  // prepare image output
  cv::Mat output_image;
  if (publish_images)
    output_image = cv_ptr->image.clone();

  geometry_msgs::PoseArray pose_array;
  
  // go through detected targets
  for (int i = 0; i < system->targets; i++) {
    const whycon::CircleDetector::Circle& circle = system->get_circle(i);
    whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
    cv::Vec3f coord = pose.pos;

    // draw each target
    if (publish_images) {
      std::ostringstream ostr;
      ostr << std::fixed << std::setprecision(2);
      ostr << coord << " " << i;
      circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
      cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(255,0,255), 1, CV_AA);
    }

    if (publish_poses) {
      geometry_msgs::Pose p;
      p.position.x = pose.pos(0);
      p.position.y = pose.pos(1);
      p.position.z = pose.pos(2);
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
      pose_array.poses.push_back(p);
    }
  }

  if (publish_images) {
    cv_bridge::CvImage output_image_bridge = *cv_ptr;
    output_image_bridge.image = output_image;
    image_pub.publish(output_image_bridge);
  }

  if (publish_poses) {
    pose_array.header = header;
    pose_array.header.frame_id = frame_id;
    poses_pub.publish(pose_array);
  }
}
