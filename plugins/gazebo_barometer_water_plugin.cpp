#include <gazebo_barometer_water_plugin.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(BarometerWaterPlugin)

    BarometerWaterPlugin::BarometerWaterPlugin() : ModelPlugin(),
                                                   baro_rnd_y2_(0.0),
                                                   baro_rnd_use_last_(false)
    {
    }

    BarometerWaterPlugin::~BarometerWaterPlugin()
    {
        update_connection_->~Connection();
    }

    void BarometerWaterPlugin::getSdfParams(sdf::ElementPtr sdf)
    {
        namespace_.clear();
        if (sdf->HasElement("robotNamespace"))
        {
            namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        else
        {
            gzerr << "[gazebo_barometer_water_plugin] Please specify a robotNamespace.\n";
        }
        if (sdf->HasElement("pubRate"))
        {
            pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
        }
        else
        {
            pub_rate_ = kDefaultPubRate;
            gzwarn << "[gazebo_barometer_water_plugin] Using default publication rate of" << pub_rate_ << " Hz\n";
        }
        if (sdf->HasElement("baroTopic"))
        {
            baro_topic_ = sdf->GetElement("baroTopic")->Get<std::string>();
        }
        else
        {
            baro_topic_ = kDefaultBarometerTopic;
            gzwarn << "[gazebo_barometer_water_plugin] Using default barometer topic " << baro_topic_ << ".\n";
        }
        if (sdf->HasElement("noise"))
        {
            baro_noise_ = sdf->GetElement("noise")->Get<double>();
        }
        else
        {
            baro_noise_ = kDefaultBarometerNoise;
        }
    }

    void BarometerWaterPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        getSdfParams(sdf);
        model_ = model;
        world_ = model_->GetWorld();
        last_time_ = world_->SimTime();
        last_pub_time_ = world_->SimTime();

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("Ros node for gazebo not initialized");
            return;
        }

        node_handle_ = new ros::NodeHandle("");

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&BarometerWaterPlugin::OnUpdate, this, _1));

        pub_baro_ = node_handle_->advertise<sensor_msgs::FluidPressure>(namespace_ + "/" + baro_topic_, 1);
    }

    void BarometerWaterPlugin::OnUpdate(const common::UpdateInfo &)
    {
        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_pub_time_).Double();

        if (dt > 1.0 / pub_rate_)
        {
            sensor_msgs::FluidPressure msg;
            double z_height = model_->GetLink("barometer_water_link")->WorldPose().Pos().Z();

            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            // pressure increases by 10 kPa/m water depth.
            msg.fluid_pressure = -z_height * 10000;

            // generate Gaussian noise sequence using polar form of Box-Muller transformation
            double x1, x2, w, y1;
            if (!baro_rnd_use_last_)
            {
                do
                {
                    x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
                    x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
                    w = x1 * x1 + x2 * x2;
                } while (w >= 1.0);
                w = sqrt((-2.0 * log(w)) / w);
                // calculate two values - the second value can be used next time because it is uncorrelated
                y1 = x1 * w;
                baro_rnd_y2_ = x2 * w;
                baro_rnd_use_last_ = true;
            }
            else
            {
                // no need to repeat the calculation - use the second value from last update
                y1 = baro_rnd_y2_;
                baro_rnd_use_last_ = false;
            }

            // apply noise.
            double noise = baro_noise_ * y1;
            msg.fluid_pressure += noise;

            pub_baro_.publish(msg);
        }
    }
} // namespace gazebo
