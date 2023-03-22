#include "set_point_of_interest_tool.h"
#include <navigation_assistant/nav_assistant_point.h>

#include <ros/console.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

namespace POI
{

    SetPointOfInterestTool::SetPointOfInterestTool()
    {
        // Set shorcut key
        shortcut_key_ = 'p';
        nav_assist_srv_client = nh_.serviceClient<navigation_assistant::nav_assistant_point>("/navigation_assistant/add_point_of_interest");
        shift_down = false;
    }

    void SetPointOfInterestTool::onInitialize()
    {
    }

    void SetPointOfInterestTool::activate()
    {
    }

    void SetPointOfInterestTool::deactivate()
    {
    }

    // Handling key events
    // ^^^^^^^^^^^^^^^^^^^^^
    int SetPointOfInterestTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
    {
        if(event->type() == QKeyEvent::KeyPress)
        {
            if( event->key() == Qt::Key_Shift )
                shift_down = true;
            else if ( event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
            {
                //Save graph
                navigation_assistant::nav_assistant_point srv_call;
                srv_call.request.action = "save";
                nav_assist_srv_client.call(srv_call);
            }
        }
        else //if(event->type() == QKeyEvent::KeyRelease)
        {
            if( event->key() == Qt::Key_Shift )
                shift_down = false;
        }
        return 1;
    }

    // Handling mouse events
    // ^^^^^^^^^^^^^^^^^^^^^
    int SetPointOfInterestTool::processMouseEvent(rviz::ViewportMouseEvent& event)
    {
        // processMouseEvent() is sort of the main function of a Tool, because
        // mouse interactions are the point of Tools.

        if (event.leftDown())   //Add/remove a new Interesting Point (space)
        {
            //1. Get (x,y,yaw) in the plane z=0
            Ogre::Vector3 intersection;
            Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
            if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
            {
                navigation_assistant::nav_assistant_point srv_call;
                if (event.shift())
                    srv_call.request.action = "delete";
                else
                    srv_call.request.action = "add";
                srv_call.request.type = "space";
                srv_call.request.pose.pose.position.x = intersection.x;
                srv_call.request.pose.pose.position.y = intersection.y;
                srv_call.request.pose.pose.position.z = 0.0;
                srv_call.request.pose.pose.orientation.w = 1.0;
                //ROS_INFO("[POI]: Requesting the creation of new [%s] at (%.3f, %.3f)  ", srv_call.request.type.c_str() , intersection.x, intersection.y );

                nav_assist_srv_client.call(srv_call);
            }
        }        
        else if (event.rightDown()) //Add a new Interesting Point (CNP)
        {
            //1. Get (x,y,yaw) in the plane z=0
            Ogre::Vector3 intersection;
            Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
            if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
            {
                navigation_assistant::nav_assistant_point srv_call;
                if (event.shift())
                    srv_call.request.action = "delete";
                else
                    srv_call.request.action = "add";
                srv_call.request.type = "CNP";
                srv_call.request.pose.pose.position.x = intersection.x;
                srv_call.request.pose.pose.position.y = intersection.y;
                srv_call.request.pose.pose.position.z = 0.0;
                srv_call.request.pose.pose.orientation.w = 1.0;
                //ROS_INFO("[POI]: Requesting the creation of new [%s] at (%.3f, %.3f)  ", srv_call.request.type.c_str() , intersection.x, intersection.y );
                nav_assist_srv_client.call(srv_call);
            }
        }
        else if (event.middleDown())
        {
        }
        return 1;
    }

} //end namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(POI::SetPointOfInterestTool, rviz::Tool )
