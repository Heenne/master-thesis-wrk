/*
  RRTstar_ros.cpp
*/
#include <rrt_star_global_planner/RRTstar_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRTstar_planner::RRTstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace RRTstar_planner
{
  RRTstarPlannerROS::RRTstarPlannerROS() : initialized_(false) 
  {
     ROS_INFO("RRT* DEFAULT CONSTRUCTOR");
  }

  RRTstarPlannerROS::RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :  initialized_(false)
  {
     ROS_INFO("RRT* OVERLOADED CONSTRUCTOR");
     initialize(name, costmap_ros);
  }

  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
  
    if (!initialized_)
    {
      ROS_INFO("Initializing RRT* planner.");
      costmap_ = costmap_ros->getCostmap();
      frame_id_ = costmap_ros->getGlobalFrameID();
  
      ros::NodeHandle private_nh("~/" + name);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      this->point_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker", 1000);

  
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
	    resolution = costmap_->getResolution();
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();

      PROBABILITY_THRESHOLD = 0.9;
      RADIUS = 1.0;
      GOAL_TOLERANCE = 0.2;
      epsilon_min = 0.1;
      epsilon_max = 0.2;
      MAX_NUM_NODES = 20000;

      initialized_ = true;
      ROS_INFO("RRT* planner initialized successfully");
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // record start time for planning to compute time taken later
    auto plan_start = std::chrono::steady_clock::now();

    // Check if the planner has been initialized
    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    
    std::vector<std::pair<float, float>> path;
    std::vector<Node> nodes;

    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;
    nodes.push_back(start_node);
    
    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;
    std::chrono::microseconds sampling_total{0};
    std::chrono::microseconds getNew_total{0};
    std::chrono::microseconds optimization_total{0};

    Node node_nearest;
    while (nodes.size() < MAX_NUM_NODES)
    {
      // ROS_INFO_STREAM("1");
      bool found_next = false;
      while (found_next == false)
      {
        // ROS_INFO_STREAM("2");
        auto sampling_start = std::chrono::steady_clock::now();
        p_rand = sampleFree(goal.pose.position.x, goal.pose.position.y); // get a random point in the free space
        // ROS_INFO_STREAM("2.1");
        auto sampling_end = std::chrono::steady_clock::now();
        sampling_total += std::chrono::duration_cast<std::chrono::microseconds>(sampling_end - sampling_start);
        auto getNew_start = std::chrono::steady_clock::now();
        node_nearest = getNearest(nodes, p_rand); // The nearest node of the random point
        // ROS_INFO_STREAM("2.2");
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate
        auto getNew_end = std::chrono::steady_clock::now();
        getNew_total += std::chrono::duration_cast<std::chrono::microseconds>(getNew_end - getNew_start);
        // ROS_INFO_STREAM("2.3");
        if (!isPointCollision(p_new.first, p_new.second))
        {
          // ROS_INFO_STREAM("3");
          Node newnode;
          newnode.x = p_new.first;
          newnode.y = p_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimization
          auto optimization_start = std::chrono::steady_clock::now();

          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent node for the new generated node
          nodes.push_back(newnode);
          nodes = rewire(nodes, newnode); 

          auto optimization_end = std::chrono::steady_clock::now();
          optimization_total += std::chrono::duration_cast<std::chrono::microseconds>(optimization_end - optimization_start);

          found_next = true;
        }

        // int counter = 0;
        // visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();
        // for(auto nodes : nodes)
        // {
        //   visualization_msgs::Marker start_position;
        //   start_position.id = counter;
        //   start_position.header.frame_id="map";
        //   start_position.header.stamp = ros::Time::now();
        //   start_position.type = visualization_msgs::Marker::CUBE;
        //   start_position.lifetime = ros::Duration(0);
        //   start_position.pose.position.x = nodes.x;
        //   start_position.pose.position.y = nodes.y;
        //   start_position.pose.position.z = 0.0;
        //   start_position.pose.orientation.x = 0.0;
        //   start_position.pose.orientation.y = 0.0;
        //   start_position.pose.orientation.z = 0.0;
        //   start_position.pose.orientation.w = 1.0;
        //   start_position.color.r = 1.0;
        //   start_position.color.a = 1.0;
        //   start_position.scale.x = 1.0;
        //   start_position.scale.y = 1.0;
        //   start_position.scale.z = 1.0;

        //   marker_array.markers.push_back(start_position);
        //   counter++;
        // }
        // point_pub_.publish(marker_array);


        // ROS_INFO_STREAM(nodes.size());

        

      }

      // Check if the new node reach the goal point with the distance less than the GOAL_TOLERANCE
      if (goalReach(p_new.first, p_new.second, goal.pose.position.x , goal.pose.position.y))
      {
        ROS_INFO("Time cost of sampling: %f ms", sampling_total.count()/1000.0);
        ROS_INFO("Time cost of finding new node: %f ms", getNew_total.count()/1000.0);
        ROS_INFO("Time cost of optimization: %f ms", optimization_total.count()/1000.0);

        std::pair<float, float> point;
        
        // New goal inside of the goal tolerance
        Node new_goal_node = nodes[nodes.size() - 1];
        Node current_node = new_goal_node;

        current_node = new_goal_node;
        // Final Path
        while (current_node.parent_id != -1)
        {
          point.first = current_node.x;
          point.second = current_node.y;
          path.insert(path.begin(), point); 
      
          current_node = nodes[current_node.parent_id];
        }

        if(path.size() > 1)
        {

          // build spline from waypoints and optimize to fulfil curvature limit
          int costmap_size_x = this->costmap_->getSizeInCellsX();
          int costmap_size_y = this->costmap_->getSizeInCellsY();
          cv::Mat costmap_img_orig = cv::Mat::zeros(costmap_size_x, costmap_size_y, CV_8UC1);
          int obstacle_count = 0;
          for (int i = 0; i < costmap_size_x; i++)
          {
              for (int j = 0; j < costmap_size_y; j++)
              {
                  double cell_cost = static_cast<double>(this->costmap_->getCost(i, j));
                  if (cell_cost > 0 || cell_cost == -1)
                  {
                      costmap_img_orig.at<uchar>(i, j, 0) = 255;
                      obstacle_count++;
                  }
              }
          }
          
          // get costmap as image
          cv::Mat costmap_img;
          cv::threshold(~costmap_img_orig, costmap_img, 1 / 0.05 / 10.0, 255, cv::THRESH_BINARY_INV);
          costmap_img.convertTo(costmap_img, CV_8UC1);

          cv::Mat obstacle_img = costmap_img;

          std::vector<cv::Point2d> sparse_path_world;
          for(auto point: path)
          {
            cv::Point2d new_point;
            new_point.x = point.first;
            new_point.y = point.second;
            sparse_path_world.push_back(new_point);
          }

          std::vector<cv::Point2d> continuous_path;
          std::vector<cv::Point2d> optimized_sparse_path;
          std::vector<double> optimized_lengths;
          costmap_2d::Costmap2D costmap_copy = *(this->costmap_);
          bool splining_success = path_smoothing::buildOptimizedContinuousPath(sparse_path_world,
                                                                              continuous_path,
                                                                              optimized_sparse_path,
                                                                              optimized_lengths,
                                                                              costmap_copy,
                                                                              obstacle_img,
                                                                              100.0,
                                                                              50,
                                                                              true,
                                                                              5.0);
          ROS_INFO_STREAM("Optimization Status: " << splining_success);
          // ROS_INFO_STREAM(continuous_path.size());
          // std_msgs::Float64MultiArray optimized_lengths_msg;
          // optimized_lengths_msg.data = optimized_lengths;
          // optimized_lengths_pub_.publish(optimized_lengths_msg);

          // // create plan from optimized sparse path for visualization
          // // std::vector<geometry_msgs::PoseStamped> optimized_sparse_plan;
          // // path_planning::createPlanFromPath(optimized_sparse_path, optimized_sparse_plan, this->frame_id_);
          // // this->optimized_sparse_path_ = optimized_sparse_plan;
          // // this->spline_tangent_lengths_ = optimized_lengths;
          // // this->publishPlan(optimized_sparse_plan, this->optimized_plan_pub_);

          if (!splining_success)
          {
              return false;
          }

          std::vector<geometry_msgs::PoseStamped> splined_plan;
          path_planning::createPlanFromPath(continuous_path, splined_plan, this->frame_id_);
          // // std::chrono::steady_clock::time_point splining_time = std::chrono::steady_clock::now();
          // // double splining_duration = (std::chrono::duration_cast<std::chrono::milliseconds>(splining_time - plan_creating_time).count()) / 1000.0;
          // // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken for building splined plan (s): " << splining_duration);

          // if (!path_planning::isPlanFree(std::shared_ptr<costmap_2d::Costmap2D>(this->costmap_), 1, splined_plan))
          // {
          //     ROS_ERROR("Plan is not free!");
          //     return false;
          // }
          // // std::chrono::steady_clock::time_point free_check_time = std::chrono::steady_clock::now();
          // // ROS_INFO_STREAM(
          // //     "SplinedVoronoiPlanner: Time taken for checking if final plan is free (s): "
          // //     << (std::chrono::duration_cast<std::chrono::microseconds>(free_check_time - splining_time).count()) /
          // //            1000000.0);
          
          // // fill resulting plan
          plan.clear();
          for (auto pose : splined_plan)
          {
            plan.push_back(pose);
          }
          if (plan.empty())
          {
              ROS_ERROR("Got empty plan from spline interpolation");
              return false;
          }
        }

        publishPlan(plan);
        return true;

    
        // //if the global planner find a path, publish it
        // plan.clear();  //clear the plan, just in case it is not empty
        // if (path.size() > 0)
        // {
        //   auto plan_end = std::chrono::steady_clock::now();
        //   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start);
        //   ROS_INFO("RRT* Global Planner: Path found!!!!");
        //   ROS_INFO("Time cost of whole RRT* path planning: %d ms", duration.count());
        //   plan.push_back(start);

        //   // convert the points to poses
        //   ros::Time plan_time = ros::Time::now();
        //   for (int i = 0; i < path.size(); i++)
        //   {
        //     geometry_msgs::PoseStamped pose;
        //     pose.header.stamp = plan_time;
        //     pose.header.frame_id = frame_id_;
        //     pose.pose.position.x = path[i].first;
        //     pose.pose.position.y = path[i].second;
        //     plan.push_back(pose);
        //   }
        //   plan.push_back(goal);
        //   publishPlan(plan);
          
        //   return true;
        // }
        // else
        // {
        //   ROS_WARN("RRT* planner failed to find a path, no elements in the path");
        //   return false;
        // }
      }
    }
    ROS_WARN("RRT* planner failed to find a path, try to increase the max number of iterations");
    return false;
  }

  bool RRTstarPlannerROS::goalReach(float x1, float y1, float x2, float y2)
  {
    float dist = distance(x1, y1, x2, y2);
    if (dist < GOAL_TOLERANCE)
      return true;
    else
      return false;
  }

  float RRTstarPlannerROS::distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
    return dist;
  }

  std::pair<float, float> RRTstarPlannerROS::sampleFree(float x_goal, float y_goal)
  {
    std::pair<float, float> random_point;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> probability (0.0, 1.0);
    if (probability(gen) > PROBABILITY_THRESHOLD)
    {
      random_point.first = x_goal;
      random_point.second = y_goal;
      return random_point;
    }
    else
    {
      for (int i = 0; i < 10000; i++)
      {
        float map_width = this->map_width_;
        float map_height = this->map_height_;

        std::uniform_real_distribution<> x(-map_width, map_width);
        std::uniform_real_distribution<> y(-map_height, map_height);

        random_point.first = x(gen);
        random_point.second = y(gen);

        if (!isPointCollision(random_point.first, random_point.second))
        {
          return random_point;
        }
      }
    }
  }

  void RRTstarPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
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

  bool RRTstarPlannerROS::isPointCollision(float wx, float wy)
  {
    unsigned int mx, my;

    if(this->costmap_->worldToMap(wx, wy, mx, my))
    {
      unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(mx, my));
      if (cell_cost > 0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    return true;
  }
  
  Node RRTstarPlannerROS::getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand)
  {
    Node node = nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second) < distance(node.x, node.y, p_rand.first, p_rand.second))
        node = nodes[i];
    }
  
    return node;
  }

  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {
    

    for (int i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < RADIUS &&
         nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y) &&
         isLineCollision(nodes[i], nn.x, nn.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    newnode.parent_id = nn.node_id;
  
    return newnode;
  }

  std::vector<Node> RRTstarPlannerROS::rewire(std::vector<Node> nodes, Node newnode)
  {
    Node node;
    for (int i = 0; i < nodes.size(); i++)
    {
      node = nodes[i];
      if (node != nodes[newnode.parent_id] && distance(node.x, node.y, newnode.x, newnode.y) < RADIUS &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost && isLineCollision(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
      }
    }
    return nodes;
  }

  std::pair<float, float> RRTstarPlannerROS::steer(float x1, float y1, float x2, float y2)
  {
    std::pair<float, float> p_new;
    float dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max && dist > epsilon_min)
    {
      p_new.first = x1;
      p_new.second = y1;
      return p_new;
    }
    else
    {
      float theta = atan2(y2-y1, x2-x1);
      p_new.first = x1 + epsilon_max*cos(theta);
      p_new.second = y1 + epsilon_max*sin(theta);
      return p_new;
    }
  }

  bool RRTstarPlannerROS::isLineCollision(Node node_nearest, float px, float py)
  {
    int n = 1;
    float theta;

    std::pair<float, float> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    float dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution)
    {
      if (isPointCollision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist/resolution));
      float theta;
      for (int i = 0;i < value; i++)
      {
        theta = atan2(node_nearest.y - py, node_nearest.x - px);
        p_n.first = node_nearest.x + n*resolution*cos(theta);
        p_n.second = node_nearest.y + n*resolution*sin(theta);
        if (isPointCollision(p_n.first, p_n.second))
          return false;
        
        n++;
      }
      return true;
    }
  }

}; // RRTstar_planner namespace
