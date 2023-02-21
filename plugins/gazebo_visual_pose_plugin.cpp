#include <gazebo_visual_pose_plugin.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(VisualPosePlugin)

    VisualPosePlugin::VisualPosePlugin()
    : ModelPlugin()
    , name_extension_("::visual_target")
    {}

    VisualPosePlugin::~VisualPosePlugin()
    {
        update_connection_->~Connection();
        if (node_handle_ != nullptr) delete node_handle_;
    }

    void VisualPosePlugin::Init()
    {
        target_model_ = world_->ModelByName(model_->GetName() + name_extension_);

        if (target_model_ == NULL) gzwarn << "target_model_ is 'NULL'!\n";
    }

    void VisualPosePlugin::getSdfParams(sdf::ElementPtr sdf)
    {
        namespace_.clear();
        if (sdf->HasElement("robotNamespace"))
        {
            namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        else
        {
            gzerr << "[gazebo_visual_pose_plugin] Please specify a robotNamespace.\n";
        }

        target_topic_.clear();
        if (sdf->HasElement("target_topic"))
        {
            target_topic_ = sdf->GetElement("target_topic")->Get<std::string>();
        }
        else
        {
            gzwarn << "[gazebo_visual_pose_plugin] Please specify target_topic\n";
        }
    }

    void VisualPosePlugin::makeVisualSDF(sdf::ElementPtr sdf)
    {
        if (sdf->GetName() == "model") sdf->GetElement("static")->Set(true);
        sdf::ElementPtr elem = sdf->GetFirstElement();

        while (elem) {
            if (elem->GetName() == "model" || elem->GetName() == "link") {
                makeVisualSDF(elem);
                if (!elem->HasElement("model") && !elem->HasElement("link") && !elem->HasElement("visual")) {
                    goto delete_and_continue;
                }
            } else if (elem->GetName() == "visual") {
                elem->GetElement("cast_shadows")->Set(false);
                elem->GetElement("transparency")->Set(0.8);
            } else if (elem->GetName() != "pose"
                    && elem->GetName() != "frame"
                    && elem->GetName() != "static") {
                goto delete_and_continue;
            }

            elem = elem->GetNextElement();
            continue;

            delete_and_continue:
            {
                sdf::ElementPtr next = elem->GetNextElement();
                elem->RemoveFromParent();
                elem = next;
                continue;
            }
        }
    }

    sdf::SDFPtr VisualPosePlugin::getVisualSDF(const physics::ModelPtr _model, const std::string& _name)
    {
        sdf::SDFPtr sdf{new sdf::SDF()};
        sdf->Version("1.6");
        sdf->Root(_model->GetSDF()->Clone());
        sdf->Root()->GetAttribute("name")->Set(_name);
        makeVisualSDF(sdf->Root());
        return sdf;
    }

    void VisualPosePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        getSdfParams(sdf);

        if (target_topic_.empty()) return;

        model_ = model;
        world_ = model_->GetWorld();

        world_->InsertModelSDF(*getVisualSDF(model_, model_->GetName() + name_extension_));

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("Ros node for gazebo not initialized");
            return;
        }

        if (namespace_.empty())
            node_handle_ = new ros::NodeHandle(model_->GetName());
        else
            node_handle_ = new ros::NodeHandle(namespace_ + "/" + model_->GetName());

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&VisualPosePlugin::OnUpdate, this, _1));

        has_new_pose = false;
        
        target_sub_ = node_handle_->subscribe(target_topic_, 1, &VisualPosePlugin::callback, this);
    }

    void VisualPosePlugin::OnUpdate(const common::UpdateInfo &)
    {
        if (target_model_ == NULL) {
            target_model_ = world_->ModelByName(model_->GetName() + name_extension_);
        
            if (target_model_ == NULL) {
                gzwarn << "target_model_ is 'NULL'!\n";
                return;
            }
        }
        if (!has_new_pose) return;

        const ignition::math::Vector3d pos(latest_pose.position.x,
                                           latest_pose.position.y,
                                           latest_pose.position.z);

        const ignition::math::Quaterniond rot(latest_pose.orientation.w,
                                              latest_pose.orientation.x,
                                              latest_pose.orientation.y,
                                              latest_pose.orientation.z);

        if (!rot.IsFinite()) return;

        const ignition::math::Pose3d pose(pos, rot.Normalized());

        target_model_->SetWorldPose(pose);

        has_new_pose = false;
    }

    void VisualPosePlugin::callback(const geometry_msgs::PoseStamped& msg)
    {
        latest_pose = msg.pose;
        has_new_pose = true;
    }
} // namespace gazebo
