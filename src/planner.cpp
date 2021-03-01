#include "ros/ros.h"
#include <stack>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "rrt_planner/rrt.hpp"


class Planner
{
private:
  // ROS stuff
  ros::NodeHandle _nh;
  ros::Subscriber _map_sub;
  ros::Subscriber _init_sub;
  ros::Subscriber _goal_sub;
  ros::Publisher _path_pub;
  ros::Publisher _marker_pub;

  // constants
  double _goal_bias;
  double _delta;

  // RRT
  RRT _rrt;
  Node _init, _goal;
  bool _ready[3] = {false, false, false};  // init, goal, map
  bool _success = false;
  nav_msgs::OccupancyGrid _map;
  int _iteration = 0;
  int _num_edges = 0;
  
  nav_msgs::Path _path;

  // rviz marker publishing functions
  void add_vertex(Node p)
  {
    visualization_msgs::Marker vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    vertex.header.frame_id = "map";
    vertex.header.stamp = ros::Time::now();
    if (p.id == 0)  // init
    {
      vertex.ns = "init/goal";
      vertex.id = 0;
      vertex.color.a = vertex.color.r = 1.0f;
      vertex.scale.x = vertex.scale.y = 0.3;
    }
    else if (p.id == -1)  // goal
    {
      vertex.ns = "init/goal";
      vertex.id = 1;
      vertex.color.a = vertex.color.g = 1.0f;
      vertex.scale.x = vertex.scale.y = 0.3;
    }
    else  // noraml vertex
    {
      vertex.ns = "vertices";
      vertex.id = p.id;
      vertex.color.a = vertex.color.b = 1.0f;
      vertex.scale.x = vertex.scale.y = 0.1;
    }
    vertex.action = visualization_msgs::Marker::ADD;

    vertex.points.push_back(p.point);

    _marker_pub.publish(vertex);
  }

  void add_edge(geometry_msgs::Point p1, geometry_msgs::Point p2)
  {
    visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = _num_edges;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.05;
    edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    _marker_pub.publish(edge);
    _num_edges++;
  }

  void clear_markers()
  {
    ros::Rate loop_rate(250);
    for (int i = 0; i < _num_edges; i++)
    {
      visualization_msgs::Marker edge;
      edge.header.frame_id = "map";
      edge.ns = "edges";
      edge.id = i;
      edge.action = visualization_msgs::Marker::DELETE;
      _marker_pub.publish(edge);
      visualization_msgs::Marker vertex;
      vertex.header.frame_id = "map";
      vertex.ns = "vertices";
      vertex.id = i;
      vertex.action = visualization_msgs::Marker::DELETE;
      _marker_pub.publish(vertex);
      loop_rate.sleep();
    }
  }

  void send_path()
  {
    std::stack<Node> path_nodes;
    std::vector<Node> node_list = _rrt.get_node_list();
    path_nodes.push(_goal);
    _path.poses.clear();

    while (path_nodes.top().parent_id >= 0)
    {
      path_nodes.push(node_list[path_nodes.top().parent_id]);
    }

    while (!path_nodes.empty())
    {
      geometry_msgs::PoseStamped p;
      p.pose.position = path_nodes.top().point;
      path_nodes.pop();
      _path.poses.push_back(p);
    }
    _path.header.frame_id = "map";

    _path_pub.publish(_path);
  }

  Node iterate()
  {
    Node next_node;
    geometry_msgs::Point target_point;
    Node nearest_node;
  
    // decide whether to expand to the goal or a random point
    do
    {
      double r = rand() / (double) RAND_MAX;
      if (r < _goal_bias)  // try to expand to goal
      {
        target_point = _goal.point;
      }
      else  // try to expand to rand_point
      {
        target_point = _rrt.get_random_config();  
      }
      nearest_node = _rrt.get_nearest_node(target_point);
      r = rand() / (double) RAND_MAX;
      if (r < _goal_bias)
      {
        next_node = _rrt.expand(nearest_node, _goal.point);
      }
      else
      {
        next_node = _rrt.expand(nearest_node, target_point);
      }
      // next_node = _rrt.expand(nearest_node, target_point);
    } while (next_node.id == nearest_node.id);  // not returning parent

    ROS_INFO("Target_point: [%f, %f]\nnearest_node: [%f, %f]\nnext_node: [%f, %f]\n",
              target_point.x, target_point.y, nearest_node.point.x,
              nearest_node.point.y, next_node.point.x, next_node.point.y);
    add_vertex(next_node);
    add_edge(nearest_node.point, next_node.point);
    _iteration++;
    return next_node;
  }

public:
  Planner(double goal_bias, double delta)
  : _goal_bias(goal_bias), _delta(delta)
  {
    // ROS subscriber and publishers
    _map_sub = _nh.subscribe<nav_msgs::OccupancyGrid>
        ("/map", 10, &Planner::map_cb, this);

    _init_sub = _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
        ("/initialpose", 10, &Planner::init_cb, this);

    _goal_sub = _nh.subscribe<geometry_msgs::PoseStamped>
        ("/move_base_simple/goal", 10, &Planner::goal_cb, this);

    _path_pub = _nh.advertise<nav_msgs::Path>("/path", 10);

    _marker_pub = _nh.advertise<visualization_msgs::Marker>
        ("/visualization_marker", 10);
  }

  void start()
  {
    _success = false;
    _iteration = 0;
    _num_edges = 0;
    _rrt = RRT(
      _init, _goal, _delta, _map.info, _map.data);
    ros::Rate loop_rate(100);

    // Check if init and goal are valid
    if (!(_rrt.is_free(_init.point) &&_rrt.is_free(_goal.point)))
    {
      ROS_WARN("Invalid init or goal point, try again!");
      _ready[0] = false;
      _ready[1] = false;
      return;
    }
    
    while (ros::ok() && !_success)
    {
      ROS_INFO("Iteration: %d", _iteration);
      Node next_node = iterate();
      if ((_rrt.get_distance(next_node.point, _goal.point) <= _delta) &&
          !_rrt.is_blocked(next_node.point, _goal.point))
      {
        add_edge(next_node.point, _goal.point);
        next_node.children.push_back(_goal);
        _goal.parent_id = next_node.id;
        send_path();
        _success = true;
        ROS_INFO("Found Path!");
        _ready[0] = false;
        _ready[1] = false;
      }
      loop_rate.sleep();
    }
  }

  // ROS callback functions
  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    _map = *msg;
    _ready[2] = true;
  }

  void init_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (!_ready[0])
    {
      _init.point = msg->pose.pose.position;
      _init.id = 0;
      _init.parent_id = -1;
      _ready[0] = true;
      ROS_INFO("Set initial pose successfully!");
      add_vertex(_init);

      if (!_ready[1])
      {
        clear_markers();
        ROS_INFO("Waiting for goal...");
      }
    }
    else
    {
      ROS_INFO("Initial pose already set! Ignoring.");
    }
  }

  void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (!_ready[1])
    {
      _goal.point = msg->pose.position;
      _goal.id = -1;
      _ready[1] = true;
      ROS_INFO("Set goal pose successfully!");
      add_vertex(_goal);

      if (!_ready[0])
      {
        clear_markers();
        ROS_INFO("Waiting for initial pose...");
      }
    }
    else
    {
      ROS_INFO("Goal already set! Ignoring.");
    }
  }

  bool is_ready()
  {
    return (_ready[0]) && (_ready[1]) && (_ready[2]);
  }

  bool is_success()
  {
    return _success;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_planner");
  double goal_bias = (argc >= 2) ? atof(argv[1]) : 0.3;
  double delta = (argc >= 3) ? atof(argv[2]) : 1.0;
  Planner planner = Planner(goal_bias, delta);
  ros::Rate rate(20);
  ROS_INFO("Planner started! Please send init pose and goal in rviz.");

  while (ros::ok() && !planner.is_ready())
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("RRT ready! Starting...");
  while (ros::ok())
  {
    if (planner.is_ready())
    {
      planner.start();
    }
    else
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
}