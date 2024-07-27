#include "nscc_cover_node.hpp"

NSCCCoverNode::NSCCCoverNode()
    : Node("nscc_cover_node")
{
    loadConfig();
    sub_polygon_map_ = create_subscription<scc_message::msg::PolygonMap>("polygon_map", 1, std::bind(&NSCCCoverNode::polygonMapCallback, this, std::placeholders::_1));
    pub_grid_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 1);
    pub_grid_map_->publish(gmap);
}

void NSCCCoverNode::loadConfig()
{
    declare_parameter("scene_width", 12.0);
    declare_parameter("scene_height", 12.0);
    declare_parameter("position_resolution", 0.05);

    scene_width_ = get_parameter("scene_width").as_double();
    scene_height_ = get_parameter("scene_height").as_double();
    position_resolution_ = get_parameter("position_resolution").as_double();

    gmap.info.width = scene_width_ / position_resolution_;
    gmap.info.height = scene_height_ / position_resolution_;
    gmap.info.resolution = position_resolution_;
    gmap.info.origin.position.x = -scene_width_ / 2;
    gmap.info.origin.position.y = -scene_height_ / 2;
    gmap.info.origin.position.z = 0;
    gmap.info.origin.orientation.x = 0;
    gmap.info.origin.orientation.y = 0;
    gmap.info.origin.orientation.z = 0;
    gmap.info.origin.orientation.w = 1;
    gmap.header.frame_id = "map";
    gmap.data.resize(gmap.info.width * gmap.info.height, 0);
}

void NSCCCoverNode::polygonMapCallback(const scc_message::msg::PolygonMap::SharedPtr msg)
{
    pmap = *msg;
    
    std::vector<cv::Point2f> boundary;
    for (auto &point : pmap.boundary.points)
    {
        boundary.push_back(cv::Point2f(point.x, point.y));
    }
    std::vector<std::vector<cv::Point2f>> holes;
    for (auto &hole : pmap.obstacles)
    {
        std::vector<cv::Point2f> hole_points;
        for (auto &point : hole.points)
        {
            hole_points.push_back(cv::Point2f(point.x, point.y));
        }
        holes.push_back(hole_points);
    }

    for (int i = 0; i < gmap.info.width; i++)
    {
        for (int j = 0; j < gmap.info.height; j++)
        {
            cv::Point2f point(gmap.info.origin.position.x + i * gmap.info.resolution, gmap.info.origin.position.y + j * gmap.info.resolution);
            if (cv::pointPolygonTest(boundary, point, false) < 0)
            {
                gmap.data[j * gmap.info.width + i] = 100;
            }
            else
            {
                gmap.data[j * gmap.info.width + i] = 0;
                for (auto &hole : holes)
                {
                    if (cv::pointPolygonTest(hole, point, false) >= 0)
                    {
                        gmap.data[j * gmap.info.width + i] = 100;
                        break;
                    }
                }
            }
        }
    }

    pub_grid_map_->publish(gmap);
}
