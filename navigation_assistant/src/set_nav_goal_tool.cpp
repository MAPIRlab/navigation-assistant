#include <tf/transform_listener.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "set_nav_goal_tool.h"

#include <navigation_assistant/nav_assistantActionGoal.h>


namespace rviz
{
    SetNavGoalTool::SetNavGoalTool()
    {
        // Here we set the "shortcut_key_" member variable defined in the
        // superclass to declare which key will activate the tool.
        shortcut_key_ = 'n';

        topic_property_ = new StringProperty( "Topic", "/nav_assistant/goal",
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
      pub_ = nh_.advertise<navigation_assistant::nav_assistantActionGoal>( topic_property_->getStdString(), 1 );
    }


    // Set what to do on pose set!
    void SetNavGoalTool::onPoseSet(double x, double y, double theta)
    {
        // get global frame
        std::string fixed_frame = context_->getFixedFrame().toStdString();

        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        geometry_msgs::PoseStamped goal;
        tf::poseStampedTFToMsg(p, goal);
        ROS_INFO("[NavAssistantTool]Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);

        navigation_assistant::nav_assistantActionGoal nav_goal;
        nav_goal.header.frame_id = fixed_frame;
        nav_goal.header.stamp = ros::Time::now();
        nav_goal.goal.target_pose = goal;
        nav_goal.goal.turn_before_nav = true;
        pub_.publish(nav_goal);
    }

}// end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SetNavGoalTool, rviz::Tool )
