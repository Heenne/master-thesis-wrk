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
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg, vector<point> & path, float & path_clearance)
{
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

  std::vector<float> path_clearance_vector;
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
    path_clearance_vector.push_back(min_distance);
  }
  path_clearance = accumulate(path_clearance_vector.begin(),path_clearance_vector.end(),0.0)/path_clearance_vector.size();
  std::cout<<"The path clearance is: "<<path_clearance<<std::endl;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "data_analysis_node");
  ros::NodeHandle nh;

  // get position(x and y) of plan path points points from .csv file
  vector<point> plan;
  ifstream infile; 
  infile.open("/home/rosmatch/bagfiles/plan.csv",ios::in); // open the file with read only mode
  readPlan(infile,plan);
  infile.close();
  
  // calculate the path length
  float path_length = 0.0;
  pathLength(path_length,plan);
  std::cout<<"The path length is: "<<path_length<<std::endl;

  // calculate the path smoothness
  float path_smoothness = 0.0;
  pathSmoothness(path_smoothness,plan);
  std::cout<<"The path smoothness is: "<<path_smoothness<<std::endl;

  // calculate the path clearance
  float path_clearance = 0.0;
  ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&mapCallback, _1,plan,path_clearance));
  ros::spin();

  return 0;
}
