#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>  
#include<vector> 
#include<cmath>
#include<boost/bind.hpp>
#include<algorithm>
#include<numeric>

using namespace std;

struct point
{
    float x;
    float y;
};

void readPlan(ifstream &infile, std::vector<point> & path)
{
  if (!infile)
  {
    std::cout<<"fail to open .CSV file!"<<std::endl;
    exit(1);
  }

  // read data from file line by line
  string line;
  vector<string> fields;
  while(getline(infile, line))
	{
		istringstream is(line);
    string field;
		while(std::getline(is, field, ','))
		{
			fields.push_back(field);
		}
	}

  // get data of position x and y(string) and transform it to float
  int total = fields.size();
  for (int i = (total/2)+7; i< total;i=i+10)
  {
    stringstream ssx;
    stringstream ssy;
    point p;
    ssx << fields[i];
    ssx >> p.x;
    ssy << fields[i+1];
    ssy >> p.y;
    path.push_back(p);
  }
  for (int i = 0; i< path.size();i++)
  {
    cout<<path[i].x<<","<<path[i].y<<endl;
  }
}

void readTrajectory(ifstream &infile, std::vector<point> & path)
{
  if (!infile)
  {
    std::cout<<"fail to open .CSV file!"<<std::endl;
    exit(1);
  }

  // read data from file line by line
  string line;
  vector<string> fields;
  while(getline(infile, line))
	{
		istringstream is(line);
    string field;
		while(std::getline(is, field, ','))
		{
			fields.push_back(field);
		}
	}

  // get data of position x and y(string) and transform it to float
  for (int i = 5; i< fields.size();i=i+4)
  {
    stringstream ssx;
    stringstream ssy;
    point p;
    ssx << fields[i];
    ssx >> p.x;
    ssy << fields[i+1];
    ssy >> p.y;
    path.push_back(p);
  }
}

float distance(point p1, point p2)
{
  float dis = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
  return dis;
}

void pathLength(float & path_length, vector<point>  path) 
{
  for (int i = 0; i < path.size()-1; i++)
  {
    path_length += distance(path[i],path[i+1]);
  }
}

void pathSmoothness(float & path_smoothness, vector<point>  path)
{
  float PI = 3.1415926;
  float sum = 0.0;
  int count = 0;
  for (int i = 1; i < path.size()-1; i++)
  {
    float a = distance(path[i],path[i-1]);
    float b = distance(path[i],path[i+1]);
    float c = distance(path[i-1],path[i+1]);
    if (a+b>c)
    {
      sum += acos((pow(a,2) + pow(b,2) - pow(c,2))/(2*a*b));
      count++;
    }
  }
  path_smoothness = 1.0 - (sum/(count*PI));
  std::cout << "count: " << count << std::endl;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg, vector<point> & path, float & path_safety)
{
  int map_width = msg->info.width;
  int map_height = msg->info.height;
  float map_resolution = msg->info.resolution;
  float map_origin_x = msg->info.origin.position.x;
  float map_origin_y = msg->info.origin.position.y;

  cout << "Map received" << endl;
  cout << "Map width: " << msg->info.width << endl;
  cout << "Map height: " << msg->info.height << endl;
  cout << "Map resolution: " << msg->info.resolution << endl;
  cout << "Map origin: " << msg->info.origin.position.x << ", " << msg->info.origin.position.y << endl;
  cout << "Map data: " << msg->data.size() << endl;

  std::vector<point> obstacles;
  for (int i = 0; i < msg->data.size(); i++)
  {
    if (msg->data[i] == 100)
    {
      point obstacle_point ;
      obstacle_point.x = (i%msg->info.width)*msg->info.resolution + msg->info.origin.position.x;
      obstacle_point.y = (i/msg->info.width)*msg->info.resolution + msg->info.origin.position.y;
      obstacles.push_back(obstacle_point);
    }
  }
  std::cout << "Number of obstacles: " << obstacles.size() << endl;

  std::vector<float> path_safety_vector;
  for (int i = 0; i < path.size(); i++)
  {
    float min_distance = sqrt(pow(path[i].x-obstacles[0].x,2)+pow(path[i].y-obstacles[0].y,2));
    for (int j = 1; j < obstacles.size(); j++)
    {
      float distance = sqrt(pow(path[i].x-obstacles[j].x,2)+pow(path[i].y-obstacles[j].y,2));
      if (distance<min_distance) 
      {
        min_distance = distance;
      }
    }
    path_safety_vector.push_back(min_distance);
  }
  path_safety = *min_element(path_safety_vector.begin(),path_safety_vector.end());
  float max_safety = *max_element(path_safety_vector.begin(),path_safety_vector.end());
  float average_safety = accumulate(path_safety_vector.begin(),path_safety_vector.end(),0.0)/path_safety_vector.size();
  std::cout<<"The min safety indicator of the path is: "<<path_safety<<std::endl;
  std::cout<<"The max safety indicator of the path is: "<<max_safety<<std::endl;
  std::cout<<"The average safety indicator of the path is: "<<average_safety<<std::endl;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "data_analysis_node");
  ros::NodeHandle nh;

  // get position(x and y) of plan path and trajectory points from .csv file
  vector<point> plan;
  // vector<point> trajectory;

  ifstream infile; 
  infile.open("/home/rosmatch/bagfiles/plan.csv",ios::in); // open the file with read only mode
  readPlan(infile,plan);
  infile.close();

  // infile.open("/home/rosmatch/bagfiles/trajectory.csv",ios::in); 
  // readTrajectory(infile,trajectory);
  // infile.close(); 

  std::cout<<"The number of the plan points: "<<plan.size()<<std::endl;
  // std::cout<<"The number of the trajectory points: "<<trajectory.size()<<std::endl;
  
  // calculate the length of the path
  float plan_length = 0.0;
  // float trajectory_length = 0.0;

  pathLength(plan_length,plan);
  // pathLength(trajectory_length,trajectory);
  std::cout<<"The length of the plan path is: "<<plan_length<<std::endl;
  // std::cout<<"The length of the trajectory path is: "<<trajectory_length<<std::endl;

  // calculate the smoothness of the path
  float plan_smoothness = 0.0;
  // float trajectory_smoothness = 0.0;

  pathSmoothness(plan_smoothness,plan);
  // pathSmoothness(trajectory_smoothness,trajectory);
  std::cout<<"The smoothness of the plan is: "<<plan_smoothness<<std::endl;
  // std::cout<<"The smoothness of the trajectory is: "<<trajectory_smoothness<<std::endl;

  // calculate the safety indicator of the path
  float plan_safety = 0.0;
  // float trajectory_safety = 0.0;
  std::cout<<"The safety indicator of the plan: "<<std::endl;
  ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&mapCallback, _1,plan,plan_safety));
  // ros::spinOnce();
  // std::cout<<"The safety indicator of the trajectory: "<<std::endl;
  // ros::Subscriber map_sub2 = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&mapCallback, _1,trajectory,trajectory_safety));
  ros::spin();

  return 0;
}
