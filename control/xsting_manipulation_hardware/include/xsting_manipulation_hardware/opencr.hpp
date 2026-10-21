#ifndef _OPENCR_HPP_
#define _OPENCR_HPP_

namespace robotis {
namespace xsting_manipulation_hardware {
class OpenCR{
public:
    explicit OpenCR(const uint8_t &id);
    virtual ~OpenCR();
    bool OpenCR::open_port(const std::string & usb_port);
    bool OpenCR::set_baud_rate(const uint32_t & baud_rate);
    uint16_t OpenCR::ping();
    bool OpenCR::is_connect_manipulator();
    bool OpenCR::is_connect_wheels();
    void OpenCR::play_sound(uint8_t sound) const;
    void OpenCR::imu_recalibration();
    opencr::IMU OpenCR::get_imu();
    opencr::Battery OpenCR::get_battery();
    void OpenCR::joints_torque(uint8_t onoff) const;
    void OpenCR::wheels_torque(uint8_t onoff) const;
    bool OpenCR::read_all();
    std::array<double, 2> OpenCR::get_wheel_positions();
    std::array<double, 2> OpenCR::get_wheel_velocities();
    bool OpenCR::set_wheel_velocities(const std::vector<double> & velocities);
    inline int32_t convert_radian_to_tick(
        const double & radian,
        const int32_t & max_tick,
        const int32_t & min_tick,
        const double & max_radian,
        const double & min_radian);
    inline double convert_tick_to_radian(
        const int32_t & tick,
        const int32_t & max_tick,
        const int32_t & min_tick,
        const double & max_radian,
        const double & min_radian);
    std::array<double, 4> OpenCR::get_joint_positions();
    std::array<double, 4> OpenCR::get_joint_velocities();
    double OpenCR::get_gripper_position();
    double OpenCR::get_gripper_velocity();
    bool OpenCR::set_joints_variables(
    const uint16_t & address,
    const std::array<int32_t, 4> & variables);
    bool OpenCR::set_joint_positions(const std::vector<double> & radians);
    bool OpenCR::set_joint_profile_acceleration(const std::array<int32_t, 4> & acceleration);
    bool OpenCR::set_joint_profile_velocity(const std::array<int32_t, 4> & velocity);
    bool OpenCR::set_gripper_variables(const uint16_t & address, const int32_t & variable);
    bool OpenCR::set_gripper_position(const double & meters);
    bool OpenCR::set_gripper_profile_acceleration(const int32_t & acceleration);
    bool OpenCR::set_gripper_profile_velocity(const int32_t & velocity);
    bool OpenCR::set_init_pose();
    bool OpenCR::set_zero_pose();
    bool OpenCR::set_home_pose();
    bool OpenCR::set_gripper_current();
    bool OpenCR::open_gripper();
    bool OpenCR::close_gripper();
    bool OpenCR::init_gripper();
    uint8_t OpenCR::read_byte(const uint16_t & address);
    void OpenCR::write_byte(const uint16_t & address, uint8_t data);
    void OpenCR::send_heartbeat(const uint8_t & count);

private:
    uint8_t id_;
    DynamixelSDKWrapper dxl_sdk_wrapper_;
}}}

#endif