#ifndef NSCC_COVER_HPP
#define NSCC_COVER_HPP

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <vector>
#include <map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "scc_message/msg/polygon_map.hpp"
#include "scc_message/msg/cover_task.hpp"

#include "opencv2/opencv.hpp"

namespace scc
{
enum CoverNodeSearchState
{
    NOT_VISITED = 0,
    IN_OPENSET,
    IN_CLOSESET
};

struct CoverPlanResult
{
    bool success = false;
    int iterations = 0;
    double planTime = 0;
    double cost = 0;
    std::vector<geometry_msgs::msg::Pose2D> path;
};

class CoverNode
{
public:
    cv::Point2i index;
    cv::Point2d position;
    bool isCovered;
    bool isObstacle;
    CoverNodeSearchState searchState;

    CoverNode* parent;

    double gCost;
    double hCost;
    double fCost;

    std::multimap<double, CoverNode*>::iterator it;
public:
    CoverNode();
    CoverNode(cv::Point2i index, cv::Point2d position, bool isObstacle);
    CoverNode(const CoverNode& other);
    CoverNode operator=(const CoverNode& other);
};

class CoverMap
{
public:
    double originX;
    double originY;
    double width;
    double height;
    double resolution;
    double cellSize;
    int rows;
    int cols;
    int cellRows;
    int cellCols;
    CoverNode** map;
    CoverNode** cellMap;
public:
    CoverMap();
    CoverMap(nav_msgs::msg::OccupancyGrid& gmap, double cellSize);
    CoverMap(const CoverMap& other);
    ~CoverMap();
    CoverMap& operator=(const CoverMap& other);
};

class NSCCCover
{
public:
    NSCCCover();
    ~NSCCCover();
public:
    CoverMap coverMap;
    scc_message::msg::CoverTask task;

    std::multimap<double, CoverNode*> openSet;
    std::vector<CoverNode*> closeSet;

    CoverPlanResult planResult;
public:
    void config(nav_msgs::msg::OccupancyGrid& gmap, double cellSize);
    void plan(scc_message::msg::CoverTask& task);
    void toMsg(nav_msgs::msg::Path& path_msg);
private:
    void cover(cv::Point2d start, cv::Point2d goal);
    int subCover(std::vector<cv::Point2d>& temp);
};
} // namespace scc

#endif // NSCC_COVER_HPP