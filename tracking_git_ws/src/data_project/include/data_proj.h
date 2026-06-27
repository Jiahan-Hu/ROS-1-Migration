#pragma once
/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <vector>
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <cav_srvs/ReRoute.h>
#include <cav_srvs/GetAvailableRoutes.h>
#include <cav_srvs/SetActiveRoute.h>
#include <cav_srvs/AbortActiveRoute.h>
#include <string>



namespace data_proj
{

    class Data_Proj
    {
        public:
            /**
             * \brief General entry point to begin the operation of this class
            */
            void run();

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            ros::Timer timer;

            void timerCallback(const ros::TimerEvent& te);
            void get_available_routes();
            void set_route();

            private:

            // node handles
            ros::CARMANodeHandle nh_, pnh_;


            // ros service client

            ros::ServiceClient rerout_client_,get_available_routes_client_,set_route_client_;

            // ROS publishers and subscribers

            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;

            


            // trajectory frequency
            double traj_freq = 10;
            double current_speed_ = 0;

            int timer_index=0;
            int start_processing_=5;

            std::string route_file_name;
            
            // initialize this node
            void initialize();

            /**
             * \brief Callback for the pose subscriber, which will store latest pose locally
             * \param msg Latest pose message
             */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
            
            /**
             * \brief Callback for the twist subscriber, which will store latest twist locally
             * \param msg Latest twist message
             */
            void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);

            
    
    };
}