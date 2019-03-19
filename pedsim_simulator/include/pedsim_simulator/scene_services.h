/**
*
* Ronja Güldenring
*
*/

#ifndef _scene_service_h_
#define _scene_service_h_

#include <ros/ros.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_srvs/SpawnPed.h>
#include <std_srvs/SetBool.h>

  /**
   * @class SceneServices
   * @brief It provides services to spawn and remove pedestrians dynamically.
   * 
   */
class SceneServices {
  // Constructor and Destructor
 public:
  SceneServices();
  virtual ~SceneServices() = default;

 protected:
   ros::NodeHandle nh_;

 private:
  int last_id_;                               //Keeping track of cluster id, that increases in pedsim
  std::string mir_flatlans_path_;               // path to mir_flatland folder

  ros::ServiceServer spawn_ped_service_;                // Service to spawn pedestrian
   /**
     * @brief Spawns Pedestrians in pedsim and flatland
     */
  bool spawnPed(pedsim_srvs::SpawnPed::Request &request,
                                pedsim_srvs::SpawnPed::Response &response);

  ros::ServiceServer remove_all_peds_service_;          // Service to remove all pedestrians
    /**
   * @brief Removes all Pedestrians in pedsim and flatland
   */
  bool removeAllPeds(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response);
  
  // Flatland services that are used to forward agent information
  ros::ServiceClient spawn_agents_;             //Service client to spawn agent in flatland
  ros::ServiceClient delete_agents_;            // Service client to remove agent in flatland.
};

#endif /* _scene_service_h_ */
