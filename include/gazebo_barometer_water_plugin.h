#ifndef gazebo_barometer_water_plugin
#define gazebo_barometer_water_plugin
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <sensor_msgs/FluidPressure.h>
#include <ros/ros.h>

namespace gazebo
{
    static constexpr auto kDefaultPubRate = 50;
    static constexpr auto kDefaultBarometerTopic = "baro";
    static constexpr auto kDefaultBarometerNoise = 100.0;

    class BarometerWaterPlugin : public ModelPlugin
    {
    public:
        BarometerWaterPlugin();
        virtual ~BarometerWaterPlugin();

    protected:
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        virtual void OnUpdate(const common::UpdateInfo &);
        void getSdfParams(sdf::ElementPtr sdf);

    private:
        std::string namespace_;
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        event::ConnectionPtr update_connection_;
        std::string baro_topic_;
        double baro_noise_;

        ros::NodeHandle *node_handle_;
        ros::Publisher pub_baro_;

        sensor_msgs::FluidPressure baro_msg_;
        unsigned int pub_rate_;

        std::default_random_engine random_generator_;
        std::normal_distribution<double> standard_normal_distribution_;

        common::Time last_pub_time_;
        common::Time last_time_;

        double alt_home_;

        double baro_rnd_y2_;
        bool baro_rnd_use_last_;
    };
} // namespace gazebo

#endif
