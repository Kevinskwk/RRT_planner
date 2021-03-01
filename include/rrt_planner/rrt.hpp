#include <geometry_msgs/Point.h>
#include <nav_msgs/MapMetaData.h>
#include <random>
#include <vector>
#include <math.h>

#define NODE_LIST_RESERVE_SIZE 1000
#define PI 3.1415926535
#define OCCUPIED_THRESH 50

struct Node
{
public:
  Node() {}
  
  Node(geometry_msgs::Point p) : point(p) {}

  int id;
  geometry_msgs::Point point;
  std::vector<Node> children;
  int parent_id;
};


class RRT
{
public:
  RRT();

  RRT(Node init, Node goal, double delta,
      nav_msgs::MapMetaData map_info, std::vector<int8_t> map);

  geometry_msgs::Point get_random_config();

  Node get_nearest_node(geometry_msgs::Point p);

  double get_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);

  Node expand(Node parent, geometry_msgs::Point goal);

  bool is_free(geometry_msgs::Point p);

  bool is_blocked(geometry_msgs::Point p1, geometry_msgs::Point p2);

  std::vector<Node> get_node_list();

private:
  Node _init, _goal;
  double _delta;
  int _width;
  int _height;
  double _res;
  std::vector<int8_t> _map;
  std::vector<Node> _node_list;
};