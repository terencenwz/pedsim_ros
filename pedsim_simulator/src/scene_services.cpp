/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include <flatland_msgs/SpawnModel.h>
#include <flatland_msgs/DeleteModel.h>
#include <iostream>
#include <ros/package.h>




SceneServices::SceneServices(){
  //Pedsim service
  spawn_ped_service_ =
      nh.advertiseService("pedsim_simulator/spawn_ped", &SceneServices::spawnPed, this);
  remove_all_peds_service_ = nh.advertiseService("pedsim_simulator/remove_all_peds", &SceneServices::removeAllPeds, this);
  
  //flatland service clients
  spawn_model_topic = ros::this_node::getNamespace() + "/spawn_model";
  spawn_agents_ = nh.serviceClient<flatland_msgs::SpawnModel>(spawn_model_topic, true);
  delete_model_topic = ros::this_node::getNamespace() + "/delete_model";
  delete_agents_ = nh.serviceClient<flatland_msgs::DeleteModel>(delete_model_topic, true);
  flatland_path_ = ros::package::getPath("flatland_setup");

  // initialize values
  last_id_ = 0;
  
}


bool SceneServices::spawnPed(pedsim_srvs::SpawnPed::Request &request,
                                pedsim_srvs::SpawnPed::Response &response) {
      const double x = request.pos.x;
      const double y = request.pos.y;
      const int n = request.number_of_peds;
      const double dx = 2;
      const double dy = 2;
      const int type = request.type;
      AgentCluster* agentCluster = new AgentCluster(x, y, n);
      agentCluster->setDistribution(dx, dy);

      agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
      for(int i = 0; i < request.waypoints.size(); i++){
        QString id;
        id.sprintf("%d_%d", request.id, i); 
        const double x = request.waypoints[i].x;
        const double y = request.waypoints[i].y;
        const double r = request.waypoints[i].z;

        // Waypoint behaviour always set to SIMPLE --> ToDo: Parametrize
        AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
        w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
        SCENE.addWaypoint(w);
        agentCluster->addWaypoint(w);
      }
      SCENE.addAgentCluster(agentCluster);

      //Spawning agents in flatland
      for (int i = last_id_+1; i <= last_id_+n; i++){
        std::string name = "person_" + std::to_string(request.id);
        std::string ns = "pedsim_agent_" +  std::to_string(i);
        flatland_msgs::SpawnModel srv;
        srv.request.yaml_path = flatland_path_ + "/objects/person.model.yaml";
        srv.request.name = name;
        srv.request.ns = ns;
        srv.request.pose.x = x;
        srv.request.pose.y = y;
        while(!spawn_agents_.isValid()){
          spawn_agents_ = nh.serviceClient<flatland_msgs::SpawnModel>(spawn_model_topic, true);
        }
        spawn_agents_.call(srv);
        if (!srv.response.success)
        {
            ROS_ERROR("Failed to create agent with the id: %d", i);
        }else{
            ROS_WARN("Spawn %s in flatland", name.c_str());
        }
      }
      last_id_+=n;
      response.finished = true;
      return true;
}

bool SceneServices::removeAllPeds(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response){
    //Remove all agents
    QList<Agent*> agents = SCENE.getAgents();
    int num_agents = agents.size();

    int count = 1;
    for(Agent* a:agents){
      int i = a->getId();
      //We don't want to delete the robot agent
      if(i == 0){
        continue;
      }

      //Deleting pedestrian in flatland
      flatland_msgs::DeleteModel srv;
      srv.request.name = "person_" + std::to_string(count);
      while(!delete_agents_.isValid()){
        delete_agents_ = nh.serviceClient<flatland_msgs::DeleteModel>(delete_model_topic, true);
      }
      delete_agents_.call(srv);
      if (!srv.response.success)
      {
          ROS_ERROR("Failed to delete agent with the id: %d", i);
          continue;
      }else{
          ROS_WARN("Removed %s", srv.request.name.c_str());
      }

      // Deleting pedestrian and waypoints in SCENE
      for(Waypoint* w: a->getWaypoints()){
        SCENE.removeWaypoint(w);
      }
      SCENE.removeAgent(a);
      count++;

      // If all agents are deleted, stop.
      if(count == num_agents){
        break;
      }
    }

    // Remove AgentCluster (shouldn't exist...)
    for(AgentCluster* a:SCENE.getAgentClusters()){
      SCENE.removeAgentCluster(a);
    }
    // Remove Attractions (shouldn't exist...)
    for(AttractionArea* a:SCENE.getAttractions()){
      SCENE.removeAttraction(a);
    }
    response.success = true;
    return true;
}