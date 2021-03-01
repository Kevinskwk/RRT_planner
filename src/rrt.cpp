#include "rrt_planner/rrt.hpp"

RRT::RRT() {}

RRT::RRT(Node init, Node goal, double delta,
         nav_msgs::MapMetaData map_info, std::vector<int8_t> map)
: _init(init), _goal(goal), _delta(delta), _width(map_info.width),
  _height(map_info.height), _res(map_info.resolution), _map(map) 
{
  _node_list.reserve(NODE_LIST_RESERVE_SIZE);
  _node_list.push_back(init);
}

geometry_msgs::Point RRT::get_random_config()
{
  geometry_msgs::Point point;
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<int> distrx(0, _width);
  std::uniform_int_distribution<int> distry(0, _height);

  point.x = distrx(generator) * _res;
  point.y = distry(generator) * _res;

  return point;
}

Node RRT::get_nearest_node(geometry_msgs::Point p)
{
  int num_nodes = _node_list.size();
  if (num_nodes == 1)
  {
    return _node_list[0];
  }

  double d_min = MAXFLOAT;
  int nearest_node;
  
  for (int i = 0; i < num_nodes; i++)
  {
    Node curr_node = _node_list[i];
    double d = get_distance(p, curr_node.point);
    if (d < d_min)
    {
      d_min = d;
      nearest_node = i;
    }
  }
  return _node_list[nearest_node];
}

double RRT::get_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
}

Node RRT::expand(Node parent, geometry_msgs::Point goal)
{
  if (get_distance(parent.point, goal) > _delta)
  {
    // get the new point
    double theta = atan2((goal.y - parent.point.y), (goal.x - parent.point.x));

    goal.y = _delta * sin(theta) + parent.point.y;
    goal.x = _delta * cos(theta) + parent.point.x;
    goal.z = 0;
  }

  // check if the point is within an obstacle
  if (!is_blocked(parent.point, goal))
  {
    Node p(goal);
    p.id = _node_list.size();
    p.parent_id = parent.id;
    parent.children.push_back(p);
    _node_list.push_back(p);
    return p;
  }
  return parent;
}

bool RRT::is_free(geometry_msgs::Point p)
{
  if (p.x / _res > _width || p.y / _res > _height) return false;

  int i = static_cast<int>(p.y / _res) * _width + static_cast<int>(p.x / _res);
  if (_map[i] >= OCCUPIED_THRESH) return false;

  return true;
}

bool RRT::is_blocked(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  // Bresenham-like algorithm
  int num_step = static_cast<int>(get_distance(p1, p2) / _res)  + 1;
  double dx = (p2.x - p1.x) / num_step;
  double dy = (p2.y - p1.y) / num_step;
  // sample points
  geometry_msgs::Point pa = p1;
  geometry_msgs::Point pb = p1;

  for (int i = 0; i < num_step; i++)
  {
    pa.x += dx;
    pb.y += dy;
    if (!is_free(pa) || !is_free(pb)) return true;

    pa.y += dy;
    pb.x += dx;
    if (!is_free(pa)) return true;
  }
  return false;
}

std::vector<Node> RRT::get_node_list()
{
  return _node_list;
}
