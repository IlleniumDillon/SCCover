#include "PolygonMap.hpp"

#include <string>

#include <QEvent>
#include <QKeyEvent>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

#include "tf2/utils.h"
#include "opencv2/opencv.hpp"

namespace scc_rviz_plugin
{
PolygonMap::PolygonMap()
    : PolygonSelectTool(), status(0), qos_profile_(1), type(0)
{
    pub_topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "polygon_map",
        "The topic on which to publish polygons.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    pub_qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        pub_topic_property_, qos_profile_);
}

PolygonMap::~PolygonMap() = default;

void PolygonMap::onInitialize()
{
    PolygonSelectTool::onInitialize();
    pub_qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("PolygonMap");
    updateTopic();
}

void PolygonMap::activate()
{
    pointColor = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f);
    lineColor = Ogre::ColourValue(0.0f, 0.5f, 0.0f, 1.0f);
    status = 0;
    type = 0;
    PolygonSelectTool::activate();
    setStatus(
        "<b>Key Q:</b> Set boundary "
        "<b>Key W:</b> Set obstacle "
        "<b>LMB:</b> Set point "
        "<b>RMB:</b> Next polygon "
        "<b>MMB:</b> Cancel"
    );
}
void PolygonMap::deactivate()
{
    PolygonSelectTool::deactivate();
}

int PolygonMap::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
{
    if (status == 1) return 0;
    if (event->key() == Qt::Key_Q)
    {
        RVIZ_COMMON_LOG_INFO_STREAM("Set boundary");
        type = 0;
        pointColor = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f);
        lineColor = Ogre::ColourValue(0.0f, 0.5f, 0.0f, 1.0f);
        return Render;
    }
    else if (event->key() == Qt::Key_W)
    {
        RVIZ_COMMON_LOG_INFO_STREAM("Set obstacle");
        type = 1;
        pointColor = Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f);
        lineColor = Ogre::ColourValue(0.5f, 0.0f, 0.0f, 1.0f);
        return Render;
    }
    return 0;
}

void PolygonMap::onPointSet(double x, double y)
{
    status = 1;
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0;
    logPoint("Point", point, "map");
}

int PolygonMap::onPolygonSet(std::vector<Ogre::Vector3> &p)
{
    if (type == 0)
    {
        msg.boundary.points.clear();
        for (auto &point : p)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            msg.boundary.points.push_back(p);
        }
        status = 0;
        publisher_->publish(msg);
        return (Render);
    }
    else if (type == 1)
    {
        geometry_msgs::msg::Polygon obstacle;
        obstacle.points.clear();
        for (auto &point : p)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            obstacle.points.push_back(p);
        }
        msg.obstacles.push_back(obstacle);
        status = 0;
        publisher_->publish(msg);
        return (Render);
    }
}

void PolygonMap::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->
        template create_publisher<scc_message::msg::PolygonMap>(
        pub_topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
    RVIZ_COMMON_LOG_INFO_STREAM("PolygonMap: pubTopic: " << pub_topic_property_->getStdString());
}

}  // namespace scc_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scc_rviz_plugin::PolygonMap, rviz_common::Tool)