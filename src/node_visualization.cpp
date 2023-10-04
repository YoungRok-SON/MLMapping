#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mlmapping/awareness.h>
#include <mlmapping/localmap.h>
#include <msg_awareness.h>
#include <msg_localmap.h>
#include <rviz_vis.h>

// 여기 있는 애들은 다 클래스를 포인터로 선언한 것
// https://stackoverflow.com/questions/40811040/c-is-class-name-a-class-pointer
// 그냥 클래스를 incomplete type로 선언해서 포인터를 저장해 놓는듯 (전방 선언해놓으려고)
awareness_map_cylindrical* awareness_map;
local_map_cartesian*       local_map;
rviz_vis*              awareness_map_rviz_pub;
rviz_vis*              local_map_rviz_pub;

// Input  : cosnt 형식의 awareness_map_msg 포인터
// Output : void
// Brief
// awareness_map_msgs의 포인터를 받아서 msg_awareness를 awareness_map으로 옮겨주고 그걸 다시 rviz에서 볼 수 있는 형식으로 바꿔 퍼블리쉬
void awarenessmap_msg_callback(const mlmapping::awarenessConstPtr awareness_map_msg)
{
  // awareness_map_msg에 있는 데이터 awareness_map로 다 옮겨줌
  msg_awareness::unpack(awareness_map_msg,awareness_map); 
  // 이렇게 옮겨주는 이유는 뭘까
  // 그냥 아래 pub_awareness_map에 바로 넘겨주면 안되는건가?

  // awareness_map에 있는 데이터를 Rviz에 띄울 수 있는 형태로 바꿔서 다시 퍼블리쉬 해버림
  awareness_map_rviz_pub->pub_awareness_map(awareness_map,awareness_map_msg->header.stamp);
}


// Brief
// local_map_msg 포인터 받아서 rviz에 맞는 토픽으로 바꿔서 퍼블리쉬
void localmap_msg_callback(const mlmapping::localmapConstPtr local_map_msg)
{
  // 데이터 복사하는 과정 똑같
  msg_localmap::unpack(local_map_msg,local_map);
  // Rviz에서 보여줄 수 있는 형태로 변경해서 퍼블리쉬
  local_map_rviz_pub->pub_local_map(local_map,local_map_msg->header.stamp);
}

// Brief
int main(int argc, char **argv)
{
  ros::init(argc, argv, "visulization_node");
  ros::NodeHandle nh;
  string configFilePath = "/home/youngmoney/catkin_ws/src/MLMapping/launch/config/l515_t265.yaml";
  // nh.getParam("/mlmapping_configfile",   configFilePath); //this is commented by YR
  //init map and publisher
  //awareness_map
  awareness_map = new awareness_map_cylindrical();
  awareness_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Rho"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Phi_deg"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Z"),
                          getIntVariableFromYaml   (configFilePath,"mlmapping_am_n_Rho"),
                          getIntVariableFromYaml   (configFilePath,"mlmapping_am_n_Z_below"),
                          getIntVariableFromYaml   (configFilePath,"mlmapping_am_n_Z_over"),
                          false);

  awareness_map->map_tmp.release(); // 얘는 왜 릴리즈 해버리는 거지?
  awareness_map_rviz_pub =  new rviz_vis();
  awareness_map_rviz_pub->set_as_awareness_map_publisher(nh,"/awareness_map",getStringFromYaml(configFilePath,"awareness_frame_id"),3,awareness_map);
  ros::Subscriber sub1 = nh.subscribe("/mlmapping_awareness", 1, awarenessmap_msg_callback);

  
  //local_map
  local_map = new local_map_cartesian();
  local_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_d_xyz"),
                      static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_xy")),
                      static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_z")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_min")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_max")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_hit")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_miss")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_occupied_sh")),
                      getBoolVariableFromYaml(configFilePath,"use_exploration_frontiers"));
  local_map->allocate_memory_for_local_map();
  local_map_rviz_pub = new rviz_vis();
  local_map_rviz_pub->set_as_local_map_publisher(nh,"/local_map",
                                                  getStringFromYaml(configFilePath,"local_frame_id"),
                                                  5,
                                                  local_map);
  ros::Subscriber sub2 = nh.subscribe("/mlmapping_local", 1, localmap_msg_callback);
  ros::spin();

  return 0;
}


