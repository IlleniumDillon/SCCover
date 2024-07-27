#include "_PolygonSelectTool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace scc_rviz_plugin
{
PolygonSelectTool::PolygonSelectTool()
    : rviz_common::Tool()
{
    pointColor = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f);
    lineColor = Ogre::ColourValue(0.0f, 0.5f, 0.0f, 1.0f);
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

PolygonSelectTool::~PolygonSelectTool() = default;

void PolygonSelectTool::onInitialize()
{
    point_positions_.clear();
    current_point_position_ = Ogre::Vector3::ZERO;
    point_markers_.clear();
    line_markers_.clear();
}

void PolygonSelectTool::activate()
{
    point_positions_.clear();
    current_point_position_ = Ogre::Vector3::ZERO;
    point_markers_.clear();
    line_markers_.clear();

    current_point_marker_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
    current_point_marker_->setColor(1.0,1.0,0.0,1.0);
    current_point_marker_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));
    current_point_marker_->setPosition(current_point_position_);
    current_point_marker_->setOffset(Ogre::Vector3(0, 0, 0.05));
    current_point_marker_->getRootNode()->setVisible(true);
}

void PolygonSelectTool::deactivate()
{
    current_point_marker_->getRootNode()->setVisible(false);
    point_positions_.clear();
    point_markers_.clear();
    line_markers_.clear();
}

int PolygonSelectTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
{
    auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

    if (event.leftDown()) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button pressed");
        return processMouseLeftButtonPressed(point_projection_on_xy_plane);
    } 
    else if (event.type == QEvent::MouseMove) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Mouse moved");
        return processMouseMoved(point_projection_on_xy_plane);
    } 
    else if (event.leftUp()) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button released");
        return processMouseLeftButtonReleased();
    }
    else if (event.rightDown())
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Right mouse button pressed");
        return processMouseRightButtonPressed();
    }
    else if(event.middleDown())
    {
        return (Finished | Render);
    }

    return 0;
}

void PolygonSelectTool::logPoint(std::string designation, geometry_msgs::msg::Point position, std::string frame)
{
    RVIZ_COMMON_LOG_INFO_STREAM(designation << " point: (" << position.x << ", " << position.y << ", " << position.z << ") in frame " << frame);
}

int PolygonSelectTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    point_positions_.push_back(xy_plane_intersection.second);
    onPointSet(xy_plane_intersection.second.x, xy_plane_intersection.second.y);
    auto point_marker = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
    point_marker->setColor(pointColor);
    point_marker->setScale(Ogre::Vector3(0.1, 0.1, 0.1));
    point_marker->setPosition(xy_plane_intersection.second);
    point_marker->setOffset(Ogre::Vector3(0, 0, 0.05));
    point_marker->getRootNode()->setVisible(true);
    point_markers_.push_back(point_marker);
    if (point_positions_.size() > 1)
    {
        auto line_marker = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
        line_marker->setColor(lineColor);
        line_marker->setPoints(point_positions_.at(point_positions_.size() - 2), point_positions_.at(point_positions_.size() - 1));
        line_markers_.push_back(line_marker);
    }
    return Render;
}

int PolygonSelectTool::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    current_point_position_ = xy_plane_intersection.second;
    current_point_marker_->setPosition(current_point_position_);
    return Render;
}

int PolygonSelectTool::processMouseLeftButtonReleased()
{
    return Render;
}

int PolygonSelectTool::processMouseRightButtonPressed()
{
    int flag = onPolygonSet(point_positions_);

    point_positions_.clear();
    point_markers_.clear();
    line_markers_.clear();

    return flag;
}

void PolygonSelectTool::showMarker(bool show)
{
   
}

} // namespace scc_rviz_plugin  
