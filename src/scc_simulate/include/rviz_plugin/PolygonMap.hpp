#ifndef POLYGON_MAP_HPP
#define POLYGON_MAP_HPP

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// #include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "_PolygonSelectTool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "scc_message/msg/polygon_map.hpp"

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace scc_rviz_plugin
{
class PolygonMap : public PolygonSelectTool
{
    Q_OBJECT
public:
    PolygonMap();
    ~PolygonMap() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;
protected:
    void onPointSet(double x, double y) override;
    int onPolygonSet(std::vector<Ogre::Vector3> & p) override;
private Q_SLOTS:
    void updateTopic();
private:
    rclcpp::Publisher<scc_message::msg::PolygonMap>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty * pub_topic_property_;
    rviz_common::properties::QosProfileProperty * pub_qos_profile_property_;

    rclcpp::QoS qos_profile_;

    int status;
    int type;

    scc_message::msg::PolygonMap msg;
};
}  // namespace scc_rviz_plugin

#endif // POLYGON_MAP_HPP