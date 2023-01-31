/*
  PRM_ros.cpp
*/
#include <prm_global_planner/PRM_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(PRM_planner::PRMPlannerROS, nav_core::BaseGlobalPlanner)

namespace PRM_planner
{
  PRMPlannerROS::PRMPlannerROS() 
        : initialized_(false) { }

  PRMPlannerROS::PRMPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
        : costmap_ros_(costmap_ros)
  {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  void PRMPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
  
    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
  
      ros::NodeHandle private_nh("~/" + name);
  
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
	    width = costmap_->getSizeInCellsX();
	    height = costmap_->getSizeInCellsY();
	    resolution = costmap_->getResolution();
      frame_id_ = costmap_ros->getGlobalFrameID();

      NUM_SAMPLES = 5000; 
      MAX_DISTANCE = 10; 
      NUM_EDGES = 10; 
      std::cout << "total number of sampling points: " << NUM_SAMPLES << std::endl;
      std::cout << "max distance of connected edge: " << MAX_DISTANCE << std::endl;
      std::cout << "max number of connected edge: " << NUM_EDGES << std::endl;

      ROS_INFO("PRM planner initialized successfully");
      initialized_ = true;

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      point_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("prm_marker", 1000);
    }
    else
      ROS_WARN("This PRM planner has already been initialized... doing nothing");
  }

  bool PRMPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // record time cost of PRM path planning
    std::chrono::microseconds learning{0};
    std::chrono::microseconds query{0};
    std::chrono::microseconds planning{0};
    auto planning_start = std::chrono::steady_clock::now();
    
    // Check if the planner is initialized
    if (!initialized_)
    {
      ROS_ERROR("This PRM planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    //create a vector to contain all sampled points and start and goal points
    std::vector<Node> nodes;
    nodes.reserve(NUM_SAMPLES + 2); 
    std::string global_frame = frame_id_;
    // Add start point into the nodes vector
    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0; // start node id is 0
    start_node.g= 0.0;
    start_node.h= 0.0;
    start_node.f = 0.0;
    nodes.push_back(start_node);
    // Generate the sampling nodes in free space
    auto learning_start = std::chrono::steady_clock::now();
    std::pair<float, float> rand_p; 
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
      rand_p = sampleFree();
      Node new_node;
      new_node.x = rand_p.first;
      new_node.y = rand_p.second;
      new_node.node_id = i+1;  // id of sampling points start from 1
      nodes.push_back(new_node);
    }
    
    visualization_msgs::MarkerArray target_position_list;
		int counter = 0;
		for(auto node : nodes)
		{
			visualization_msgs::Marker start_position;
			start_position.id = counter;
			start_position.header.frame_id="map";
			start_position.header.stamp = ros::Time::now();
			start_position.type = visualization_msgs::Marker::CUBE;
			start_position.lifetime = ros::Duration(0);
			start_position.pose.position.x = node.x;
			start_position.pose.position.y = node.y;
			start_position.pose.position.z = 0.0;
			start_position.pose.orientation.x = 0.0;
			start_position.pose.orientation.y = 0.0;
			start_position.pose.orientation.z = 0.0;
			start_position.pose.orientation.w = 1.0;
			start_position.color.r = 1.0;
			start_position.color.a = 1.0;
			start_position.scale.x = 1.0;
			start_position.scale.y = 1.0;
			start_position.scale.z = 1.0;

			target_position_list.markers.push_back(start_position);
			counter++;
		}
		this->point_pub_.publish(target_position_list);

    // Add goal point into the nodes vector
    Node goal_node;
    goal_node.x = goal.pose.position.x;
    goal_node.y = goal.pose.position.y;
    goal_node.node_id = NUM_SAMPLES + 1;
    nodes.push_back(goal_node);
    // Find neighbours (available edges) for each node
    auto findNeighbour_start = std::chrono::steady_clock::now();

    std::vector< std::vector<int> > neighbours_list; //it stores neighbours'id for each node
    neighbours_list.reserve(NUM_SAMPLES + 2);
    for (int i = 0; i < nodes.size(); i++)
    {
      std::priority_queue<std::pair<int, float>, std::vector< std::pair<int,float> >, cmp1> node_pair_list;
      for (int j = 0; j < nodes.size(); j++)
      {
        std::pair<int, float> node_pair;
        node_pair.first = j;
        node_pair.second = GetEuclideanDistance(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y);
        node_pair_list.push(node_pair);
      }

      std::vector<int> neighbours;
      int edge_count = 0;
      while (!node_pair_list.empty() && edge_count <= NUM_EDGES)
      {
        std::pair<int, float> node_nearest = node_pair_list.top();
        if (node_nearest.second <= MAX_DISTANCE && node_nearest.first != i && !isLineCollision(nodes[i].x, nodes[i].y, nodes[node_nearest.first].x, nodes[node_nearest.first].y))
        {
          neighbours.push_back(node_nearest.first);
          node_pair_list.pop();
          edge_count = edge_count + 1;
        }
        else
        {
          node_pair_list.pop();
        }
      }
      neighbours_list.push_back(neighbours);
    }

    auto learning_end = std::chrono::steady_clock::now();
    learning = std::chrono::duration_cast<std::chrono::microseconds>(learning_end - learning_start);
    ROS_INFO("Time cost of learning phase: %f ms", learning.count()/1000.0);
    
    // Find the path using A* algorithm
    auto query_start = std::chrono::steady_clock::now();
    std::vector<std::pair<float, float>> path;
    if (findPath(nodes, neighbours_list, start_node, goal_node))
    {
      getPath(path, nodes, start_node, goal_node);
    }
    else
    {
      ROS_WARN("Failed to find a path using PRM!");
    }
    auto query_end = std::chrono::steady_clock::now();
    query = std::chrono::duration_cast<std::chrono::microseconds>(query_end - query_start);
    ROS_INFO("Time cost of query phase: %f ms", query.count()/1000.0);

    if(path.size() > 1)
    {
          // Final Path
        std::vector<geometry_msgs::PoseStamped> selected_poses;
        for (auto temp: path)
        {
          geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
          pose.pose.position.x = temp.first;
          pose.pose.position.y = temp.second;
          pose.pose.position.z = 0;
          selected_poses.push_back(pose);
        }
// Create splines
          std::vector<std::shared_ptr<bezier_splines::QuinticBezierSplines>> spline_list;
          for (int pose_counter = 0; pose_counter < (selected_poses.size() - 1); pose_counter++)
          {
            Eigen::Matrix<float, 2, 1> start_pose;
            Eigen::Matrix<float, 2, 1> end_pose;

            int next_pose_counter = 0;

            start_pose << selected_poses[pose_counter].pose.position.x, selected_poses[pose_counter].pose.position.y;
            end_pose << selected_poses[pose_counter + next_pose_counter].pose.position.x, selected_poses[pose_counter + next_pose_counter].pose.position.y;

            while(sqrt(pow(abs(start_pose(0) - end_pose(0)),2) + pow(abs(start_pose(1) - end_pose(1)),2)) < 1.0)
            {
              if (pose_counter +next_pose_counter < (selected_poses.size() - 1))
              {
                next_pose_counter++;
                start_pose << selected_poses[pose_counter].pose.position.x, selected_poses[pose_counter].pose.position.y;
                end_pose << selected_poses[pose_counter + next_pose_counter].pose.position.x, selected_poses[pose_counter + next_pose_counter].pose.position.y;
              }
              else
              {
                break;
              }
              
            }

            std::shared_ptr<bezier_splines::QuinticBezierSplines> spline =
                std::make_shared<bezier_splines::QuinticBezierSplines>(start_pose, end_pose);

            if (spline_list.size() > 0)
            {
              spline_list.back()->setNextSpline(spline);
              spline->setPreviousSpline(spline_list.back());
            }
            spline_list.push_back(spline);
          }

          // Set start tangent of first spline
          tf::Quaternion start_quaternion;
          tf::quaternionMsgToTF(start.pose.orientation, start_quaternion);
          spline_list.front()->setStartTangentByQuaternion(start_quaternion);
          // Set end tangent of last spline
          tf::Quaternion end_quaternion;
          tf::quaternionMsgToTF(goal.pose.orientation, end_quaternion);
          spline_list.back()->setEndTangentByQuaternion(end_quaternion);
          spline_list.back()->setEndTangentMagnitude(spline_list.back()->getEndTangentMagnitude() * 2.0);

          // Visualization
          // for (std::shared_ptr<bezier_splines::QuinticBezierSplines> &spline : spline_list)
          // {
            // 	spline->calcControlPoints();
            // spline->addStartEndPointToVisuHelper();
            //     spline->addTangentsToVisuHelper();
            //     spline->addControlPointsToVisuHelper();
            //     spline->addBezierSplineToVisuHelper(this->ras_params_->planning_points_per_spline);
          //   spline->visualizeData();
          // }

          // Optimize the curvature of the splines to be under a certain threshold
          // ROS_INFO_STREAM("Optimizing the curvature of the splines");
          //       ROS_INFO_STREAM("Curve Radius: " << this->ras_params_->max_robot_to_formation_centre_dist);
          for (int spline_counter = 0; spline_counter < spline_list.size(); spline_counter++)
          {
            // ROS_INFO_STREAM("spline counter: " << spline_counter);
            // ROS_INFO_STREAM("valid: " << spline_list[spline_counter]->checkMinCurveRadiusOnSpline(this->ras_params_->planning_points_per_spline, this->ras_params_->minimal_curve_radius, int i));

            int timeout_counter = 0;

            int point_of_failure = 0;
            while (!spline_list[spline_counter]->checkMinCurveRadiusOnSpline(50,
                                                                             0.5, // radius
                                                                             point_of_failure))
            {
              // This process could be turned into an iterative optimization process, that tries to get to the limit of the minimal curve radius to minimize the distance the robots have to travel
              if (point_of_failure < (50.0 / 2.0))
              {
                float current_start_magnitude = spline_list[spline_counter]->getStartTangentMagnitude();
                // ROS_INFO_STREAM("Current_start_magnitude: " << current_start_magnitude);
                spline_list[spline_counter]->setStartTangentMagnitude(1.05 * current_start_magnitude); // 1.5 ist just a value that I picked for the moment.
              }
              else
              {
                float current_end_magnitude = spline_list[spline_counter]->getEndTangentMagnitude();
                // ROS_INFO_STREAM("Current_end_magnitude: " << current_end_magnitude);
                spline_list[spline_counter]->setEndTangentMagnitude(1.05 * current_end_magnitude); // 1.5 ist just a value that I picked for the moment.
              }

              spline_list[spline_counter]->calcControlPoints();

              // Loop was not able to optimize the spline after x iterations. Maybe make parameter for this?
              if (timeout_counter >= 20)
              {
                ROS_ERROR_STREAM("SplinedRelaxedAStar: Timeout while optimizing spline, number: " << spline_counter);
                break;
              }
              timeout_counter++;
            }

            // ROS_INFO_STREAM("spline counter: " << spline_counter);
            // ROS_INFO_STREAM("valid: " << spline_list[spline_counter]->checkMinCurveRadiusOnSpline(this->planning_points_per_spline_, this->minimal_curve_radius_));
          }

          ROS_INFO_STREAM("Finished optimizing splines");

          // Create list of points as plan
          plan.clear();
          std::vector<Eigen::Vector2f> points_of_plan;

          // How much length is left to get out of the new spline (target_spline_length - old_spline_approx_length)

          for (std::shared_ptr<bezier_splines::QuinticBezierSplines> &spline : spline_list)
          {
            for (double i = 0.0; i <= 1.0; i = i + 0.01)
            {
              Eigen::Vector2f point_on_spline = spline->calcPointOnBezierSpline(i);
              points_of_plan.push_back(spline->calcPointOnBezierSpline(i));
            }
          }
          // Add the target point to the spline as it will most likely not be added
          // points_of_plan.push_back(spline_list.back()->calcPointOnBezierSpline(1.0));

          geometry_msgs::PoseStamped last_pose;
          // < is necessary because we just copy elements from one vector (0 until size()) to the other
          for (uint path_counter = 0; path_counter < points_of_plan.size(); path_counter++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = this->frame_id_;

            pose.pose.position.x = points_of_plan[path_counter][0];
            pose.pose.position.y = points_of_plan[path_counter][1];

            // Calculate orientation for each point of the plan with the current position and the last one
            if (path_counter == 0) // No previous point so orientation of start will be taken
            {
              pose.pose.orientation = start.pose.orientation;
            }
            else // Some other points are before, so orientation can be calculated
            {
              float delta_x = pose.pose.position.x - last_pose.pose.position.x;
              float delta_y = pose.pose.position.y - last_pose.pose.position.y;
              double yaw_angle = std::atan2(delta_y, delta_x);
              pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);
            }

            last_pose = pose; // Safe pose for next iteration
            plan.insert(plan.begin() + plan.size(), pose);
          }



      // // build spline from waypoints and optimize to fulfil curvature limit
      // int costmap_size_x = this->costmap_->getSizeInCellsX();
      // int costmap_size_y = this->costmap_->getSizeInCellsY();
      // cv::Mat costmap_img_orig = cv::Mat::zeros(costmap_size_x, costmap_size_y, CV_8UC1);
      // int obstacle_count = 0;
      // for (int i = 0; i < costmap_size_x; i++)
      // {
      //     for (int j = 0; j < costmap_size_y; j++)
      //     {
      //         double cell_cost = static_cast<double>(this->costmap_->getCost(i, j));
      //         if (cell_cost > 0 || cell_cost == -1)
      //         {
      //             costmap_img_orig.at<uchar>(i, j, 0) = 255;
      //             obstacle_count++;
      //         }
      //     }
      // }
      
      // // get costmap as image
      // cv::Mat costmap_img;
      // cv::threshold(~costmap_img_orig, costmap_img, 1 / 0.05 / 10.0, 255, cv::THRESH_BINARY_INV);
      // costmap_img.convertTo(costmap_img, CV_8UC1);

      // cv::Mat obstacle_img = costmap_img;

      // std::vector<cv::Point2d> sparse_path_world;
      // for(auto point: path)
      // {
      //   cv::Point2d new_point;
      //   new_point.x = point.first;
      //   new_point.y = point.second;
      //   sparse_path_world.push_back(new_point);
      // }

      // std::vector<cv::Point2d> continuous_path;
      // std::vector<cv::Point2d> optimized_sparse_path;
      // std::vector<double> optimized_lengths;
      // costmap_2d::Costmap2D costmap_copy = *(this->costmap_);
      // bool splining_success = path_smoothing::buildOptimizedContinuousPath(sparse_path_world,
      //                                                                     continuous_path,
      //                                                                     optimized_sparse_path,
      //                                                                     optimized_lengths,
      //                                                                     costmap_copy,
      //                                                                     obstacle_img,
      //                                                                     100.0,
      //                                                                     50,
      //                                                                     true,
      //                                                                     5.0);
      // ROS_INFO_STREAM("Optimization Status: " << splining_success);
      // // ROS_INFO_STREAM(continuous_path.size());
      // // std_msgs::Float64MultiArray optimized_lengths_msg;
      // // optimized_lengths_msg.data = optimized_lengths;
      // // optimized_lengths_pub_.publish(optimized_lengths_msg);

      // // // create plan from optimized sparse path for visualization
      // // // std::vector<geometry_msgs::PoseStamped> optimized_sparse_plan;
      // // // path_planning::createPlanFromPath(optimized_sparse_path, optimized_sparse_plan, this->frame_id_);
      // // // this->optimized_sparse_path_ = optimized_sparse_plan;
      // // // this->spline_tangent_lengths_ = optimized_lengths;
      // // // this->publishPlan(optimized_sparse_plan, this->optimized_plan_pub_);

      // if (!splining_success)
      // {
      //     return false;
      // }

      // std::vector<geometry_msgs::PoseStamped> splined_plan;
      // path_planning::createPlanFromPath(continuous_path, splined_plan, this->frame_id_);
      // // // std::chrono::steady_clock::time_point splining_time = std::chrono::steady_clock::now();
      // // // double splining_duration = (std::chrono::duration_cast<std::chrono::milliseconds>(splining_time - plan_creating_time).count()) / 1000.0;
      // // // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken for building splined plan (s): " << splining_duration);

      // // if (!path_planning::isPlanFree(std::shared_ptr<costmap_2d::Costmap2D>(this->costmap_), 1, splined_plan))
      // // {
      // //     ROS_ERROR("Plan is not free!");
      // //     return false;
      // // }
      // // // std::chrono::steady_clock::time_point free_check_time = std::chrono::steady_clock::now();
      // // // ROS_INFO_STREAM(
      // // //     "SplinedVoronoiPlanner: Time taken for checking if final plan is free (s): "
      // // //     << (std::chrono::duration_cast<std::chrono::microseconds>(free_check_time - splining_time).count()) /
      // // //            1000000.0);
      
      // // // fill resulting plan
      // for (auto pose : splined_plan)
      // {
      //   plan.push_back(pose);
      // }
      // if (plan.empty())
      // {
      //     ROS_ERROR("Got empty plan from spline interpolation");
      //     return false;
      // }
    }

    publishPlan(plan);
    return true;

    //create plan message
  //   plan.clear();
  //   plan.push_back(start);
  //   if (path.size() > 0)
  //   {
  //     // show planning time
  //     auto planning_end = std::chrono::steady_clock::now();
  //     planning = std::chrono::duration_cast<std::chrono::microseconds>(planning_end - planning_start);
  //     ROS_INFO("Time cost of whole PRM path planning: %f ms", planning.count()/1000.0);

  //     // convert the point coordinate to pose
  //     ros::Time plan_time = ros::Time::now();
  //     for (int i = 1; i < path.size() -1; i++)
  //     {
  //       geometry_msgs::PoseStamped pose;
  //       pose.header.stamp = plan_time;
  //       pose.header.frame_id = frame_id_;
  //       pose.pose.position.x = path[i].first;
  //       pose.pose.position.y = path[i].second;
  //       pose.pose.position.z = 0.0;
  //       pose.pose.orientation.x = 0.0;
  //       pose.pose.orientation.y = 0.0;
  //       pose.pose.orientation.z = 0.0;
  //       pose.pose.orientation.w = 1.0;
  //       plan.push_back(pose);
  //     }
  //     plan.push_back(goal);
  //     publishPlan(plan);
  //     return true;
  //   }
  //   else
  //   {
  //     ROS_WARN("Failed to find a path! There is no path elements!");
  //     return false;
  //   }
  }


  std::pair<float, float> PRMPlannerROS::sampleFree()
  {
    std::pair<float, float> random_point;
    float map_width = costmap_->getSizeInMetersX();
    float map_height = costmap_->getSizeInMetersY();
    // float map_width = 30.0;
    // float map_height = 30.0;
    bool findNode = false;
    while(!findNode)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis_x(originX, originX + map_width);
      std::uniform_real_distribution<> dis_y(originY, originY + map_height);
      // std::uniform_real_distribution<> dis_x(0.0-map_width, map_width);
      // std::uniform_real_distribution<> dis_y(0.0-map_height, map_height);
      random_point.first = dis_x(gen);
      random_point.second = dis_y(gen);
      if (!isPointCollision(random_point.first, random_point.second))
      {
        findNode = true;
      }
    }
    return random_point;
  }

  bool PRMPlannerROS::isPointCollision(float wx, float wy)
  {
    unsigned int mx, my;
    this->costmap_->worldToMap(wx, wy, mx, my);
    int cell_cost = static_cast<int>(this->costmap_->getCost(mx, my));
    if (cell_cost > 0 || cell_cost == -1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool PRMPlannerROS::isLineCollision(float px1, float py1, float px2, float py2)
  {
    float Dist = GetEuclideanDistance(px1, py1, px2, py2);
    float dx = (px2 - px1) / Dist;
    float dy = (py2 - py1) / Dist;
    float x = px1;
    float y = py1;
    float n = 0.0;
    float step = 0.1;
    while (n < Dist)
    {
      if (isPointCollision(x, y))
      {
        return true;
      }
      x += dx * step;
      y += dy * step;
      n += step;
    }

  }

  float PRMPlannerROS::GetEuclideanDistance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt(pow((px1 - px2),2) + pow((py1 - py2),2));
    return dist;
  }


  bool PRMPlannerROS::findPath(std::vector<Node>& nodes, std::vector<std::vector<int>>& neighbours_list, Node& start_point, Node& goal_point)
  {
    std::priority_queue<Node,std::vector<Node>, cmp> openList;
	  std::priority_queue<Node,std::vector<Node>, cmp> closeList;
    openList.push(start_point);
    while (!openList.empty())
    {
      // get node with lowest cost in openlist as current node
      Node current_node = openList.top();
      // remove it from openlist so that it won't be visited again
      openList.pop();
      closeList.push(current_node);
      if (current_node.node_id == goal_point.node_id)
      {
        ROS_INFO("Find the path in PRM!");
        return true;
      }
      // get all neighbours(id) of current node
      std::vector<int> current_neighbours = neighbours_list[current_node.node_id];
      if (current_neighbours.size() == 0)
      {
        ROS_WARN("Failed to find a path! There is no available edge for a certain node!");
        return false;
      }
      for (int i = 0; i < current_neighbours.size(); i++)
      {
        // get one neighbour of current node
        int neighbour_id = current_neighbours[i];
        if (isInList(neighbour_id, openList) == false && isInList(neighbour_id, closeList) == false)
        {
          nodes[neighbour_id].parent_id = current_node.node_id;
          nodes[neighbour_id].g = current_node.g + GetEuclideanDistance(current_node.x, current_node.y, nodes[neighbour_id].x, nodes[neighbour_id].y);
          nodes[neighbour_id].h = GetEuclideanDistance(nodes[neighbour_id].x, nodes[neighbour_id].y, goal_point.x, goal_point.y);
          nodes[neighbour_id].f = nodes[neighbour_id].g + nodes[neighbour_id].h;
          openList.push(nodes[neighbour_id]);
        }
        else if (isInList(neighbour_id, openList) == true)
        {
          float g_new = current_node.g + GetEuclideanDistance(current_node.x, current_node.y, nodes[neighbour_id].x, nodes[neighbour_id].y);
          if (g_new < nodes[neighbour_id].g)
          {
            nodes[neighbour_id].parent_id = current_node.node_id;
            nodes[neighbour_id].g = g_new;
            nodes[neighbour_id].f = nodes[neighbour_id].g + nodes[neighbour_id].h;
          }
        }
      }
    }
    return false;
  }

  // 
  void PRMPlannerROS::getPath(std::vector<std::pair<float, float>> & path, std::vector<Node>& nodes, Node start_pose, Node goal_pose)
  {
    Node current_point;
    current_point.node_id = goal_pose.node_id;
    current_point.x = goal_pose.x;
    current_point.y = goal_pose.y;
    while (current_point != start_pose)
    {
      for (int i = 0;i < nodes.size();i++)
      {
        if (current_point.node_id == nodes[i].node_id)
        {
          std::pair<float, float> path_point;
          path_point.first = nodes[i].x;
          path_point.second = nodes[i].y;
          path.push_back(path_point);
          int current_id = nodes[i].parent_id;
          current_point.x = nodes[current_id].x;
          current_point.y = nodes[current_id].y;
          current_point.node_id = current_id;
          break;
        }
      }
    }
    std::pair<float, float> path_point;
    path_point.first = current_point.x;
    path_point.second = current_point.y;
    path.push_back(path_point);
    std::reverse(path.begin(),path.end());
  }

  bool PRMPlannerROS::isInList(int Node_id, std::priority_queue<Node,std::vector<Node>, cmp> List)
  {
    while (!List.empty())
    {
      if (Node_id == List.top().node_id)
      {
        return true;
      }
      else 
      {
        List.pop();
      }
    }
    return false;
  }

  void PRMPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
  {
    if (plan.size() < 1)
    {
        ROS_ERROR("Plan has no elements!");
        return;
    }
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = plan;

    plan_pub_.publish(gui_path);
  }

}; // PRM_planner namespace
