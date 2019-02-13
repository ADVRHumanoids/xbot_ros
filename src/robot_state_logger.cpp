#include <vector>
#include <signal.h>
#include <future>
#include <chrono>
#include <functional>

#include <sys/wait.h>
#include <sys/select.h>
#include <sys/fcntl.h>


#include <ros/ros.h>
#include <XBotInterface/XBotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <xbot_msgs/JointState.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>


bool g_msg_received  = false;
bool g_needs_restart = false;
XBot::MatLogger2::Ptr g_logger;
XBot::MatAppender::Ptr g_appender;
XBot::XBotInterface * g_model;

void log_chain_ids(xbot_msgs::JointStateConstPtr msg)
{
    for(const auto& pair : g_model->getChainMap())
    {
        std::vector<int> ch_idx;
        for(const auto& jname : pair.second->getJointNames())
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), jname);
            if(it != msg->name.end())
            {
                ch_idx.push_back(it - msg->name.begin() + 1); // NOTE MATLAB 1-BASED INDEXING 
            }
        }
        g_logger->add(pair.first, ch_idx);
    }
}

void on_js_received(xbot_msgs::JointStateConstPtr msg)
{
    if(!g_logger)
    {
        XBot::MatLogger2::Options opt;
        opt.enable_compression = true;
	opt.default_buffer_size = 1000;
        g_logger = XBot::MatLogger2::MakeLogger("/tmp/robot_state_log", opt);
        
        g_appender->add_logger(g_logger);
        
        log_chain_ids(msg);
    }
    
    g_logger->add("link_position", msg->link_position);
    g_logger->add("motor_position", msg->motor_position);
    g_logger->add("link_velocity", msg->link_velocity);
    g_logger->add("motor_velocity", msg->motor_velocity);
    g_logger->add("stiffness", msg->stiffness);
    g_logger->add("damping", msg->damping);
    g_logger->add("effort", msg->effort);
    g_logger->add("position_reference", msg->position_reference);
    g_logger->add("velocity_reference", msg->velocity_reference);
    g_logger->add("effort_reference", msg->effort_reference);
    g_logger->add("current", msg->aux);
    g_logger->add("fault", msg->fault);
    g_logger->add("time", msg->header.stamp.toSec());
    g_logger->add("seq", msg->header.seq);
    
    g_msg_received = true;
}

bool check_host_reachable()
{
    std::string command = "ping -c 1 -W 1 " + ros::master::getHost() + " 1>/dev/null";
    return system(command.c_str()) == 0;
}


void on_timer_event(const ros::WallTimerEvent& event, 
                    std::function<void(void)> heartbeat)
{
    heartbeat();
    
    if(!g_logger && (!check_host_reachable() || !ros::master::check()))
    {
	g_logger.reset();
	heartbeat();
        std::cout << "Lost connection with master" << std::endl;
        g_needs_restart = true;
        ros::shutdown();
    }
    
    if(g_logger && !g_msg_received)
    {
	g_logger.reset();
	heartbeat();
        std::cout << "No message received from joint states" << std::endl;
        g_needs_restart = true;
        ros::shutdown();
    }
    
    g_msg_received = false;
    
}

void logger_sigint_handler(int)
{
    printf("logger: caught signal sigint\n");
    ros::shutdown();
}

int logger_main(int argc, char** argv, 
                std::function<void(void)> heartbeat)
{
    signal(SIGINT, logger_sigint_handler);
    
    ros::init(argc, argv, 
              "robot_state_logger", 
              ros::init_options::NoSigintHandler
             );
    
    std::cout << "Waiting for ROS master..." << std::endl;
    
    for(;;)
    {
        heartbeat();
        
        if(!ros::ok() || ros::isShuttingDown())
        {
            return EXIT_SUCCESS;
        }
        
        if(!check_host_reachable())
        {
            printf("Host '%s' is unreachable\n", ros::master::getHost().c_str());
            continue;
        }
        
        if(!ros::master::check())
        {
            printf("Ros master at '%s' is unreachable\n", ros::master::getURI().c_str());
            sleep(1);
            continue;
        }
        
//         try
//         {
//             XBot::ConfigOptionsFromParamServer();
//         }
//         catch(std::exception& e)
//         {
//             printf("Unable to obtain model (%s)\n", e.what());
//             sleep(1);
//             continue;
//         }
        
        break;

        
    }
    
    std::cout << "Connected to ROS master!" << std::endl;
    
    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::XBotInterface xbi;
    xbi.init(cfg);
    g_model = &xbi;
    
    ros::NodeHandle nh;
    
    ros::Subscriber js_sub =  nh.subscribe("xbotcore/joint_states",
                                           5, 
                                           on_js_received);

    ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(0.5), 
                                              boost::bind(on_timer_event, _1, heartbeat));
    
        
    g_appender = XBot::MatAppender::MakeInstance();
    g_appender->start_flush_thread();
    timer.start();
    
    ros::spin();
    
    g_appender.reset();
    g_logger.reset();
    
    printf("logger exiting with return code %d\n", g_needs_restart);

    return g_needs_restart;   
    
}

volatile sig_atomic_t g_run = 1;

void sigint_handler(int)
{
    printf("caught signal SIGINT\n");
    g_run = 0;
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigint_handler);
    
    bool restart = true;
    int logger_pid = -1;
    
    while(restart && g_run)
    {
        /* If logger pid is valid, kill it */
        if(logger_pid > 0 && waitpid(logger_pid, nullptr, WNOHANG) == 0)
        {
            printf("killing logger (pid %d)\n", logger_pid);
            kill(logger_pid, SIGKILL);
            printf("waiting logger exit..\n");
            waitpid(logger_pid, nullptr, 0);
        }
        
        /* Create pipe */
        int pipe_fd[2];
        if(pipe(pipe_fd) == -1)
        {
            perror("pipe");
        }
        auto heartbeat = [pipe_fd]()
        {
            int alive = 0;
            if(write(pipe_fd[1], &alive, sizeof(alive)) < 0)
            {
                perror("write");
            }
        };
        
        /* Fork the logger */
        logger_pid = fork();
        if(logger_pid == -1)
        {
            perror("fork");
        }
        
        if(logger_pid == 0)
        {
            close(pipe_fd[0]);
            return logger_main(argc, argv, heartbeat);
        }
        
        /* Parent process */
        close(pipe_fd[1]);
        fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK); 
        
        while(g_run)
        {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(pipe_fd[0], &fds);
            int max_fd = pipe_fd[0];
            
            timeval timeout;
            timeout.tv_sec = 2;
            timeout.tv_usec = 0;
            
            int ret = select(max_fd + 1, &fds, nullptr, nullptr, &timeout);
            
            if(ret < 0)
            {
                perror("select");
            }
            
            int status = -1;
            int pid_ret = waitpid(logger_pid, &status, WNOHANG);
            
            if(pid_ret == logger_pid && WIFEXITED(status))
            {
                int ret_code = WEXITSTATUS(status);
                printf("logger exited with return code %d\n", ret_code);
                logger_pid = -1;
                restart = ret != 0;
                break;
            }
            
            if(pid_ret == logger_pid && WIFSIGNALED(status))
            {
                int sig = WTERMSIG(status);
                printf("logger died from signal %d\n", sig);
                logger_pid = -1;
                restart = true;
                break;
            }
            
            if(ret > 0) // message received
            {
                int value = -1;
                if(read(pipe_fd[0], &value, sizeof(value)) < 0)
                {
                    perror("read");
                }
            }
            else // no message received
            {
                printf("logger is not alive\n");
                restart = true;
                break;
            }
        }
        
        close(pipe_fd[0]);
        
    }
    
    printf("waiting logger exit..\n");
    waitpid(logger_pid, nullptr, 0);
    printf("main exiting..\n");
        
    return 0;
    
    
}
