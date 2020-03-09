#include "AuxRosPublisher.h"
#include <std_msgs/Float32.h>

using namespace XBot;

AuxRosPublisher::AuxRosPublisher():
    _aux_current_idx(0),
    _name_to_aux_idx(GetAuxFieldMap())
{
    int max_aux_idx = std::max_element(_name_to_aux_idx.begin(),
                                       _name_to_aux_idx.end(),
                                       [](auto a, auto b)
    {
        return a.second < b.second;
    })->second;

    _aux_idx_tdm.reserve(10);
    _pubs.resize(max_aux_idx + 1);

}



bool XBot::AuxRosPublisher::init_control_plugin(Handle::Ptr handle)
{

    _robot = handle->getRobotInterface();

    auto rosh = handle->getRosHandle();

    // advertise topics and reserve memory
    for(auto pair : _name_to_aux_idx)
    {
        AuxPubData pub_data;
        pub_data.pub = rosh->advertise<xbot_msgs::AuxState>("/xbotcore/aux/" + pair.first, 30);
        pub_data.msg.aux.assign(_robot->getJointNum(), -1);
        pub_data.msg.name = _robot->getEnabledJointNames();
        pub_data.msg.aux_field_name = pair.first;
        _pubs[pair.second] = pub_data;
    }

    // get aux field names from yaml
    auto cfg = YAML::LoadFile(handle->getPathToConfigFile());
    if(cfg && cfg["AuxRosPublisher"] && cfg["AuxRosPublisher"]["aux_field_names"])
    {
        auto names = cfg["AuxRosPublisher"]["aux_field_names"].as<std::vector<std::string>>();

        for(auto n : names)
        {
            if(_name_to_aux_idx.count(n) == 0)
            {
                Logger::warning("[AuxRosPublisher] invalid aux field '%s' \n",
                                n.c_str());
            }
            else
            {
                Logger::info("[AuxRosPublisher] added aux field '%s' \n",
                             n.c_str());

                _aux_idx_tdm.push_back(_name_to_aux_idx[n]);
            }
        }
    }
    else
    {
        for(auto p : _name_to_aux_idx)
        {
            _aux_idx_tdm.push_back(p.second);
        }
    }

    _test = rosh->advertise<std_msgs::Float32>("/ciao", 100);
    return true;
}

void XBot::AuxRosPublisher::control_loop(double time, double period)
{
    // get current wall timestamp
    auto now = get_wall_time();

    // iter
    static int iter = 0;
    iter++;

    std_msgs::Float32 mtest;
    mtest.data = iter;
    _test->pushToQueue(mtest);

    return;

    // for each joint...
    for(int i = 0; i < _robot->getJointNum(); i++)
    {
        int jid = _robot->getEnabledJointId().at(i);

        // request aux id from multiplex list
        get_xbotcore_joint()->set_op_idx_aux(jid, _aux_idx_tdm[_aux_current_idx]);

        // get current aux id from pdo rx
        double aux_id = -1;
        get_xbotcore_joint()->get_op_idx_ack(jid, aux_id);

        // invalid aux_id, skip joint
        if(aux_id < 0 || aux_id >= _pubs.size())
        {
            continue;
        }

        // get aux field value
        double aux_value = 0.0;
        get_xbotcore_joint()->get_aux(jid, aux_value);

        // add joint name and aux value to the right publisher
        auto& pdata = _pubs[static_cast<int>(aux_id)];
        pdata.msg.aux[i] = aux_value;
        pdata.valid_data = true;

    }

    // update aux id to be requested at next iteration
    _aux_current_idx = (_aux_current_idx + 1) % _aux_idx_tdm.size();

    // publish all
    for(auto& pdata : _pubs)
    {
        if(pdata.valid_data)
        {
            pdata.msg.header.stamp = ros::Time(now.tv_sec, now.tv_nsec);
            pdata.msg.iter = iter;
            pdata.pub->pushToQueue(pdata.msg);
            pdata.valid_data = false;
            pdata.msg.aux.assign(_robot->getJointNum(), -1.0);
        }
    }

}


bool XBot::AuxRosPublisher::close()
{
}


REGISTER_XBOT_PLUGIN_(AuxRosPublisher)
