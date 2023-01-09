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
      std::cout << "originX: " << originX << std::endl;
      std::cout << "originY: " << originY << std::endl;
	    width = costmap_->getSizeInCellsX();
	    height = costmap_->getSizeInCellsY();
      std::cout << "width: " << width << std::endl;
      std::cout << "height: " << height << std::endl;
	    resolution = costmap_->getResolution();
      frame_id_ = costmap_ros->getGlobalFrameID();

      NUM_SAMPLES = 2000; 
      MAX_DISTANCE = 10; 
      NUM_EDGES = 10; 

      ROS_INFO("PRM planner initialized successfully");
      initialized_ = true;

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    }
    else
      ROS_WARN("This PRM planner has already been initialized... doing nothing");
  }

  bool PRMPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // record the start time of path planning
    double startTime = ros::Time::now().toSec();
    std::cout << "total number of sampling points: " << NUM_SAMPLES << std::endl;
    std::cout << "max_distance: " << MAX_DISTANCE << std::endl;
    std::cout << "max number of connected edge: " << NUM_EDGES << std::endl;
    
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
    std::cout << "Start node: " << start_node.x << ", " << start_node.y << std::endl;

    // Generate the sampling nodes in free space
    auto sampling_start = std::chrono::steady_clock::now();
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
    auto sampling_end = std::chrono::steady_clock::now();
    std::cout << "Sampling time: " << std::chrono::duration_cast<std::chrono::milliseconds>(sampling_end - sampling_start).count() << " ms" << std::endl;

    // Add goal point into the nodes vector
    Node goal_node;
    goal_node.x = goal.pose.position.x;
    goal_node.y = goal.pose.position.y;
    goal_node.node_id = NUM_SAMPLES + 1;
    nodes.push_back(goal_node);
    std::cout << "Goal node: " << goal_node.x << ", " << goal_node.y << std::endl;
    std::cout << "Number of sampling points: " << nodes.size() << std::endl;

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
    auto findNeighbour_end = std::chrono::steady_clock::now();
    std::cout << "Find neighbours time: " << std::chrono::duration_cast<std::chrono::milliseconds>(findNeighbour_end - findNeighbour_start).count() << " ms" << std::endl;

    // show edges coordinates
    for (int i = 0; i < neighbours_list.size(); i++)
    {
      std::cout << "Neighbours of node " << i << ": ";
      for (int j = 0; j < neighbours_list[i].size(); j++)
      {
        std::cout << neighbours_list[i][j] << " ";
      }
      std::cout << std::endl;
    }
    
    // Find the path using A* algorithm
    std::vector<std::pair<float, float>> path;
    if (findPath(nodes, neighbours_list, start_node, goal_node))
    {
      for (int i = 0; i < nodes.size(); i++)
      {
        std::cout << "parent of node " << i <<" is node: " << nodes[i].parent_id << std::endl;
      }
      getPath(path, nodes, start_node, goal_node);
      std::cout << "Path size: " << path.size() << std::endl;
        for (int j = 0; j < path.size(); j++)
      {
        std::cout << "path node x: " << path[j].first << " and y:" << path[j].second << std::endl;
      }
    }
    else
    {
      ROS_WARN("Failed to find a path using PRM!");
    }

    //create plan message
    plan.clear();
    plan.push_back(start);
    if (path.size() > 0)
    {
      // show planning time
      double endTime = ros::Time::now().toSec();
      ROS_INFO("PRM path planning time: %f", endTime - startTime);
      ros::Time plan_time = ros::Time::now();

      // convert the point coordinate to pose
      for (int i = 1; i < path.size() -1; i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = path[i].first;
        pose.pose.position.y = path[i].second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
      }
      plan.push_back(goal);
      publishPlan(plan);
      return true;
    }
    else
    {
      ROS_WARN("Failed to find a path! There is no path elements!");
      return false;
    }
  }


  std::pair<float, float> PRMPlannerROS::sampleFree()
  {
    std::pair<float, float> random_point;
    // float map_width = costmap_->getSizeInMetersX();
    // float map_height = costmap_->getSizeInMetersY();
    float map_width = 15.0;
    float map_height = 15.0;
    bool findNode = false;
    while(!findNode)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      // std::uniform_real_distribution<> dis_x(originX, originX + map_width);
      // std::uniform_real_distribution<> dis_y(originY, originY + map_height);
      std::uniform_real_distribution<> dis_x(0.0-map_width, map_width);
      std::uniform_real_distribution<> dis_y(0.0-map_height, map_height);
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
    std::cout << "start node id: " << start_point.node_id << std::endl;
    std::cout << "end node id:  " << goal_point.node_id << std::endl;
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