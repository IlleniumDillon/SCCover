#include "SCCCoverTask.hpp"

#include <string>

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
SCCCoverTask::SCCCoverTask()
    : PointSelecet(), qos_profile_(1), counter_(0)
{
    shortcut_key_ = 'g';

    pub_topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "scc_cover_task",
        "The topic on which to publish goals.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    pub_qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        pub_topic_property_, qos_profile_);
}
SCCCoverTask::~SCCCoverTask() = default;

void SCCCoverTask::onInitialize()
{
    PointSelecet::onInitialize();
    pub_qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("SCCCoverTask");
    updateTopic();
}
void SCCCoverTask::activate()
{
    PointSelecet::activate();
}
void SCCCoverTask::deactivate()
{
    PointSelecet::deactivate();
}
int SCCCoverTask::onPointSet(double x, double y)
{
    if (counter_ == 0)
    {
        msg.start.x = x;
        msg.start.y = y;
        msg.start.z = 0;
        counter_++;
        logPoint("Start", msg.start, context_->getFixedFrame().toStdString());
        return (Render);
    }
    else
    {
        msg.goal.x = x;
        msg.goal.y = y;
        msg.goal.z = 0;
        counter_ = 0;
        publisher_->publish(msg);
        logPoint("Goal", msg.goal, context_->getFixedFrame().toStdString());
        return (Finished | Render);
    }
}
void SCCCoverTask::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<scc_message::msg::CoverTask>(
        pub_topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
        RVIZ_COMMON_LOG_INFO_STREAM("SCCCoverTask: pubTopic: " << pub_topic_property_->getStdString());

}
}

// namespace scc_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scc_rviz_plugin::SCCCoverTask, rviz_common::Tool)