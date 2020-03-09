#ifndef AUXROSPUBLISHER_H
#define AUXROSPUBLISHER_H

#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XBotRosUtils.h>
#include <xbot_msgs/AuxState.h>

namespace XBot {

class AuxRosPublisher : public XBotControlPlugin
{

public:

    AuxRosPublisher();

    bool init_control_plugin(Handle::Ptr handle) override;
    void control_loop(double time, double period) override;
    bool close() override;

private:

    struct AuxPubData
    {
        RosUtils::PublisherWrapper::Ptr pub;
        xbot_msgs::AuxState msg;
        bool valid_data = false;
    };


    RobotInterface::Ptr _robot;

    std::map<std::string, int> _name_to_aux_idx;
    std::vector<int> _aux_idx_tdm;
    int _aux_current_idx;
    std::vector<AuxPubData> _pubs;

    RosUtils::PublisherWrapper::Ptr _test;



};

}

namespace
{
std::map<std::string, int> GetAuxFieldMap()
{
    return {
        {"num_elem",               0x0},
        {"round_trip_time",        0x1},
        {"pos_ref_fb",             0x2},
        {"iq_ref",                 0x3},
        {"iq_out",                 0x4},
        {"id_ref",                 0x5},
        {"id_out",                 0x6},
        {"torque_no_average",      0x7},
        {"torque_no_calibrated",   0x8},
        {"board_temperature",      0x9},
        {"motor_temperature",      0xa},
        {"battery_current",        0xb},
        {"motor_vel_filt",         0xc},
        {"motor_encoder",          0xd},
        {"link_vel_filt",          0xe},
        {"link_encoder",           0xf},
        {"deflection_encoder",     0x10},
        {"position_ref_filtered",  0x11},
        {"motor_vel_not_filtered", 0x12},
        {"motor_enc_warning",      0x13},
        {"motor_enc_errors",       0x14},
        {"link_enc_warning",       0x15},
        {"link_enc_errors",        0x16},
        {"defl_enc_warning",       0x17},
        {"defl_enc_errors",        0x18}
    };
}
}


#endif // AUXROSPUBLISHER_H
