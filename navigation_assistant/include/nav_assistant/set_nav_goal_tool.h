#ifndef SET_NAV_GOAL_H
#define SET_NAV_GOAL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include <rviz_common/properties/string_property.hpp>

#endif
#include "nav_assistant_msgs/action/nav_assistant.hpp"

namespace rviz_nav_assistant
{
    // Every tool which can be added to the tool bar is a subclass of rviz::Tool (or another Class inheriting from it)
    class RVIZ_DEFAULT_PLUGINS_PUBLIC SetNavGoalTool: public rviz_default_plugins::tools::PoseTool, public rclcpp::Node
    {
    Q_OBJECT
    public:
      SetNavGoalTool();
      virtual ~SetNavGoalTool() {}

      virtual void onInitialize();

    private:
      virtual void onPoseSet(double x, double y, double theta);

    private Q_SLOTS:
      void updateTopic();

    private:
      rclcpp::Publisher<nav_assistant_msgs::action::NavAssistant_Goal>::SharedPtr pub_;

      rviz_common::properties::StringProperty* topic_property_;      //where to publish the navigation goals.
    };

}

#endif

