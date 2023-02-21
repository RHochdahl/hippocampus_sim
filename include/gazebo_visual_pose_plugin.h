#ifndef gazebo_visual_pose_plugin
#define gazebo_visual_pose_plugin
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace gazebo
{
    class VisualPosePlugin : public ModelPlugin
    {
    public:
        VisualPosePlugin();
        virtual ~VisualPosePlugin();
        virtual void Init();

    protected:
        static sdf::SDFPtr getVisualSDF(const physics::ModelPtr _model, const std::string& _name);
        static void makeVisualSDF(sdf::ElementPtr sdf);

        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        virtual void OnUpdate(const common::UpdateInfo &);
        void getSdfParams(sdf::ElementPtr sdf);

    private:
        const std::string name_extension_;

        std::string namespace_;
        physics::ModelPtr model_;
        physics::ModelPtr target_model_;
        physics::WorldPtr world_;
        event::ConnectionPtr update_connection_;

        std::string target_topic_;

        ros::NodeHandle *node_handle_;
        ros::Subscriber target_sub_;

        transport::NodePtr visual_node_;
        transport::PublisherPtr visual_pub_;
        msgs::Visual visual_target_;

        geometry_msgs::Pose latest_pose;
        bool has_new_pose;

        void callback(const geometry_msgs::PoseStamped& msg);
    };
} // namespace gazebo

#endif
