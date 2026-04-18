#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Service
////////////////////////////////

bool TmRosNode::connect_tm(tm_msgs::srv::ConnectTM::Request::SharedPtr req, tm_msgs::srv::ConnectTM::Response::SharedPtr res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    switch (req->server) {
    case tm_msgs::srv::ConnectTM::Request::TMSVR:
        if (req->connect) {
            rb = ethernetSlaveConnection->connect(t_o);
        }
        if (req->reconnect) {
            rb = ethernetSlaveConnection->re_connect(t_o,t_v);
        }
        else {
            ethernetSlaveConnection->no_connect();
        }
        break;
    case tm_msgs::srv::ConnectTM::Request::TMSCT:
        rb = listenNodeConnection->connect_tmsct(req->timeout, req->timeval, req->connect, req->reconnect);
    }
    res->ok = rb;
    return rb;
}

bool TmRosNode::write_item(tm_msgs::srv::WriteItem::Request::SharedPtr req, tm_msgs::srv::WriteItem::Response::SharedPtr res)
{
    bool rb = false;
    std::string content = req->item + "=" + req->value;
    rb = (iface_.svr.send_content_str(req->id, content) == iface_.RC_OK);
    res->ok = rb;
    return rb;
}

bool TmRosNode::ask_item(tm_msgs::srv::AskItem::Request::SharedPtr req, tm_msgs::srv::AskItem::Response::SharedPtr res)
{
    std::lock_guard<std::mutex> lock(svr_mutex_);
    svr_response_map_[req->id] = "";
    svr_updated_ = false;
    std::string content = req->item;
    iface_.svr.send_content_str(req->id, content);
    if (req->wait_time > 0) {
        auto start_time = std::chrono::high_resolution_clock::now();
        while (!svr_updated_) {
            svr_cond_.wait_for(std::chrono::seconds(req->wait_time));
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() > req->wait_time) {
                break;
            }
        }
    }
    res->id = req->id;
    res->content = svr_response_map_[req->id];
    svr_response_map_.erase(req->id);
    svr_updated_ = false;
    return true;
}

bool TmRosNode::send_script(tm_msgs::srv::SendScript::Request::SharedPtr req, tm_msgs::srv::SendScript::Response::SharedPtr res)
{   
    bool rb = listenNodeConnection->send_listen_node_script(req->id, req->script);
    res->ok = rb;
    return rb;
}

bool TmRosNode::set_event(tm_msgs::srv::SetEvent::Request::SharedPtr req, tm_msgs::srv::SetEvent::Response::SharedPtr res)
{
    bool rb = false;
    switch (req->func) {
    case tm_msgs::srv::SetEvent::Request::EXIT:
        rb = iface_.script_exit();
        break;
    case tm_msgs::srv::SetEvent::Request::TAG:
        rb = iface_.set_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent::Request::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent::Request::STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::srv::SetEvent::Request::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::srv::SetEvent::Request::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res->ok = rb;
    return rb;
}

bool TmRosNode::set_io(tm_msgs::srv::SetIO::Request::SharedPtr req, tm_msgs::srv::SetIO::Response::SharedPtr res)
{
    bool rb = iface_.set_io(TmIOModule(req->module), TmIOType(req->type), int(req->pin), req->state);
    res->ok = rb;
    return rb;
}

bool TmRosNode::set_positions(tm_msgs::srv::SetPositions::Request::SharedPtr req, tm_msgs::srv::SetPositions::Response::SharedPtr res)
{
    bool rb = false;
    switch(req->motion_type) {
    case tm_msgs::srv::SetPositions::Request::PTP_J:
        rb = iface_.set_joint_pos_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions::Request::PTP_T:
        rb = iface_.set_tool_pose_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions::Request::LINE_T:
        rb = iface_.set_tool_pose_Line(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    }
    res->ok = rb;
    return rb;
}
bool TmRosNode::ask_sta(tm_msgs::srv::AskSta::Request::SharedPtr req, tm_msgs::srv::AskSta::Response::SharedPtr res)
{
    res->ok = listenNodeConnection->ask_sta_struct(req->subcmd, req->subdata, req->wait_time, res->subcmd, res->subdata);
    return res->ok;
}