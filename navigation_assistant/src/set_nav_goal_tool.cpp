#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "set_nav_goal_tool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nav_assistant_msgs/action/nav_assistant.hpp>


namespace rviz_nav_assistant
{
    SetNavGoalTool::SetNavGoalTool() : Node("NacAssistantRvizTool")
    {
        // Here we set the "shortcut_key_" member variable defined in the
        // superclass to declare which key will activate the tool.
        shortcut_key_ = 'n';

        topic_property_ = new rviz_common::properties::StringProperty( "Topic", "nav_assistant",
                                              "The topic on which to publish navigation assitant goals.",
                                               getPropertyContainer(), SLOT( updateTopic() ), this );
    }


    void SetNavGoalTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName( "NavAssistant Goal" );     //The name to appear on the RVIZ Tool
        updateTopic();
    }


    // Allow to set the topic name in the RVIZ properties
    void SetNavGoalTool::updateTopic()
    {
      client = rclcpp_action::create_client<nav_assistant_msgs::action::NavAssistant>(this, topic_property_->getStdString());
    }


    // Set what to do on pose set!
    void SetNavGoalTool::onPoseSet(double x, double y, double theta)
    {
        // get global frame
        std::string fixed_frame = context_->getFixedFrame().toStdString();

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = fixed_frame;
        goal.header.stamp = now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0;
        goal.pose.orientation = tf2::toMsg(quat);

        RCLCPP_INFO(get_logger(), "[NavAssistantTool]Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);

        nav_assistant_msgs::action::NavAssistant::Goal nav_goal;
        nav_goal.target_pose = goal;
        nav_goal.turn_before_nav = true;
        client->async_send_goal(nav_goal);
    }

}// end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz_nav_assistant::SetNavGoalTool, rviz_common::Tool )
