
#ifndef SET_NAV_GOAL_H
#define SET_NAV_GOAL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include <ros/ros.h>
# include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    // Every tool which can be added to the tool bar is a subclass of rviz::Tool (or another Class inheriting from it)
    class SetNavGoalTool: public PoseTool
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
      ros::NodeHandle nh_;
      ros::Publisher pub_;

      StringProperty* topic_property_;      //where to publish the navigation goals.
    };

}

#endif

