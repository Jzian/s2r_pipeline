#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose2D.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "s2r_pipeline/TargetNumber.h"
#include "Navcore.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <atomic>
#include "tr1/memory"
#include "geometry_msgs/Twist.h"


class ServiceCaller{
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    ros::NodeHandle nh;
    ros::ServiceClient Detect_client;
    ros::ServiceClient Grasp_client;
    // std::tr1::shared_ptr<boost::thread> thread_ptr_;
    std::atomic_bool srvCalling_{},srvFinished_{};
public:
    ServiceCaller();
    ~ServiceCaller() = default;
    uint32_t DetectTarget[3];
    void Detect_worker(uint32_t flag,uint32_t num);
    void Grasp_worker(uint32_t flag, uint32_t NUM);
    bool DetecDone{false}, GraspDone{false};
    bool callSrv(int SrvRequestType,Eigen::Vector3d transformation=Eigen::Vector3d(0,0,0));
    //const sim2real::sim2real_srv::Response & getSrvResponseStatus()const {return srv.response;};
    bool srvCalling() const { return srvCalling_;};
    bool srvFinished() {bool temp{srvFinished_};srvFinished_=false;return temp;};
    
};

class EP_Nav{
private:
    // ros::NodeHandle nh;
    enum GoalFlag
        {
            BOX,
            NUM1,
            NUM2,
            NUM3,
            NUM4,
            NUM5,
            DETECT,
        } goalFlag{};

    struct targetPose{
        geometry_msgs::Pose2D pose;
        
        enum TargetAction
        {
            ARRIVAL,
            UNARRIVAL,

        } targetAction{};
    };

    int NUMS = 0,NUMS_last = 0;
    geometry_msgs::Twist MoveTwist;
    geometry_msgs::Pose2D Target;
    bool TargetGetFlag{false},newGoal{true},DetectFlag{false},done{false},PutFlag{false};
    ros::Publisher log_pub;
    ServiceCaller* serviceCaller;
    ros::Subscriber subTarget;
    ros::Publisher pub_move_cmd;
    ros::Publisher base_move_vel_pub;
    NavCore::MoveBaseActionResult result{NavCore::EMPTY};
    // void TargetCallback(const std_msgs::String::ConstPtr& msg_p);
    void TargetNumberCallback(const std_msgs::String::ConstPtr& msg_p);
    void setGoal(const geometry_msgs::Pose2D &goal2d);
    void Move_cmd(geometry_msgs::Twist poseIn);
    void Move_cmd_Stop();
    bool ArrivalGoal(geometry_msgs::Pose2D PosrIn);
    
public:
    // EP_Nav(const std::string& base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf);
    // ~EP_Nav();
    enum Case 
    {
        DetectNum = 1,
        Grasp,
        Put,
    };
    
    struct Posearray{
        double x[7] , y[7] , th[7] ;
    }posearray{};
    std::string MAP_FRAME , BASE_FOOT_PRINT;
    Posearray pose_targets{};
    geometry_msgs::Pose2D ToPose(Posearray poseIn,int num);
    std::vector<int>::iterator ite;
    std::vector<int> TagetNumArray;
    std::vector<targetPose>targetPoseArray{};
    std::vector<targetPose>::iterator iter;
    NavCore *navCore;
    void GotoTarget(Posearray posearray,int goal);
    void getPoseArray();
    Posearray PoseSet();
    void run();
    EP_Nav(const std::string& base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf, ros::NodeHandle &nh)
    {
        // std::cout << "70" << std::endl;
        // printf("line:71");
        // baseController = new BaseController(serial_addr,B115200,base_foot_print,std::move(odom_frame),publish_tf);
        navCore = new NavCore(base_foot_print,std::move(map_frame));
        serviceCaller = new ServiceCaller;
        //subTarget = nh.subscribe<std_msgs::String>("/judgement/exchange_markers",10, &EP_Nav::TargetCallback,this);
        subTarget = nh.subscribe<std_msgs::String>("/target_number",10, &EP_Nav::TargetNumberCallback,this);

        pub_move_cmd = nh.advertise<geometry_msgs::Twist>("cmd_position",10);
        base_move_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        ite = TagetNumArray.begin();
                
        pose_targets = PoseSet();
        DetectFlag = false;
    }
    ~EP_Nav()
    {
        delete serviceCaller;
        delete navCore;
    }
};


    typedef enum{
        SERVICE_STATUS_SUCCEED,
        SERVICE_STATUS_EMPTY,         
    }ServiceStatus;

ServiceCaller::ServiceCaller()
{
    Detect_client = nh.serviceClient<s2r_pipeline::TargetNumber>("/detect_grasp_place_service");
    Grasp_client = nh.serviceClient<s2r_pipeline::TargetNumber>("/detect_grasp_place_service");
       
    
    
    // client =nh.serviceClient<sim2real::sim2real_srv>("manipulate");
}
void ServiceCaller::Detect_worker(uint32_t flag ,uint32_t num)
{
    std::cout << "worker" <<std::endl;
    s2r_pipeline::TargetNumber Srv;
    Srv.request.work_case = flag;
    Srv.request.number = num;
    DetecDone = Detect_client.call(Srv);
    DetectTarget[0] = Srv.response.target1;
    DetectTarget[1] = Srv.response.target2;
    DetectTarget[2] = Srv.response.target3;

    // bool flag2 = Srv.response.Detect_result;
    // std::cout << flag2 << std::endl;
    std::cout << DetectTarget[1] <<std::endl;
}

void ServiceCaller::Grasp_worker(uint32_t  flag , uint32_t NUM)
{
    std::cout << "grasp_worker" <<std::endl;
    s2r_pipeline::TargetNumber Srv;
    Srv.request.number = NUM;
    Srv.request.work_case = flag;
    Grasp_client.call(Srv);
    GraspDone = Srv.response.Detect_result;
    // bool flag2 = Srv.response.Detect_result;
    // std::cout << flag2 << std::endl;
}

bool ServiceCaller::callSrv(int SrvRequestType,Eigen::Vector3d transformation)
{
    // thread_ptr_.reset(new boost::thread(boost::bind(&ServiceCaller::worker, this, SrvRequestType,transformation)));
	// thread_ptr_->detach();
}


void EP_Nav::GotoTarget(EP_Nav::Posearray posearray , int goal)
{
    targetPose target_pose{};
    target_pose.pose.x = posearray.x[goal];
    target_pose.pose.y = posearray.y[goal];
    //target_pose.pose.theta = posearray.th[goal];
    navCore->setGoal(target_pose.pose);

}

bool EP_Nav::ArrivalGoal(geometry_msgs::Pose2D PoseTarget)
{
    geometry_msgs::Pose2D currentpose;
    double dx,dy,dth;
    currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);
    dx = std::abs(currentpose.x - PoseTarget.x);
    dy = std::abs(currentpose.y - PoseTarget.y);
    dth = currentpose.theta - PoseTarget.theta;
    
    if(dx < 0.15 && dy < 0.15)
    {
        if(!done)
        {
            navCore->cancelAllGoals();
            currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);                
            dth = currentpose.theta - PoseTarget.theta;
            MoveTwist.angular.z = -dth;
            Move_cmd(MoveTwist);
            NUMS_last++;
            done = true;    
        }    
        if(std::abs(dth) < 0.05)
            Move_cmd_Stop();
        
        if(dx < 0.2 && dy < 0.2 && std::abs(dth) < 0.2)
        {
            done = false;
            return true;
        }           
    }
    else 
        return false;
}

void EP_Nav::Move_cmd(geometry_msgs::Twist poseIn)
{
    geometry_msgs::Twist pose;
    pose.linear.x = poseIn.linear.x;
    pose.linear.y = poseIn.linear.y;
    pose.linear.z = poseIn.linear.z;
    pose.angular.x = poseIn.angular.x;
    pose.angular.y = poseIn.angular.y;
    pose.angular.z = poseIn.angular.z;
    pub_move_cmd.publish(pose);
    pose.angular.z = 1.0;
    base_move_vel_pub.publish(pose);
}

void EP_Nav::Move_cmd_Stop()
{
    geometry_msgs::Twist pose;
    pose.linear.x = 0;
    pose.linear.y = 0;
    pose.linear.z = 0;
    pose.angular.x = 0;
    pose.angular.y = 0;
    pose.angular.z = 0;
    base_move_vel_pub.publish(pose);
}

EP_Nav::Posearray EP_Nav::PoseSet()
{
    EP_Nav::Posearray posearray{};
    posearray.x[0] = 1.213 ,posearray.y[0] = 1.722, posearray.th[0] = 0.016;    //Box
    posearray.x[1] = 0.202 ,posearray.y[1] = 2.818, posearray.th[1] = 1.618;    //num1
    posearray.x[2] = 0.147 ,posearray.y[2] = 2.263, posearray.th[2] = 0.117;    //num2
    posearray.x[3] = 2.504 ,posearray.y[3] = 2.441, posearray.th[3] = 1.599;    //num3
    posearray.x[4] = 2.024 ,posearray.y[4] = 0.194, posearray.th[4] = 3.129;    //num4
    posearray.x[5] = 2.830 ,posearray.y[5] = -0.822, posearray.th[5] = -0.186;   //num5
    posearray.x[6] = 0.17 ,posearray.y[6] = 1.722, posearray.th[6] = 0.003;  //detect GoalNums
    return posearray;
}

void EP_Nav::TargetNumberCallback(const std_msgs::String::ConstPtr& msg_p)
{   
    
    //TagetNumArray.push_back(nums);
}
// void EP_Nav::TargetCallback(const std_msgs::String::ConstPtr& msg_p){
//     std::string str;
//     int target;
//     std::string charstr;
//     std::stringstream ss;
//     if (!TargetGetFlag)
//     {
//         TargetGetFlag = true;
//         str = msg_p->data.c_str();    
//         for (int i=0; i<str.length(); i++)
//         {
//             if(str[i] > 47 && str[i] < 58)
//             { 
//                 // std::cout<<"nums:" << str[i]<< std::endl;
//                 charstr = str[i]; 
//                 std::stringstream ss(charstr);
//                 ss >> target;
//                 TagetNumArray.push_back(target);
//             }
//             std::cout << "array:" << TagetNumArray.size() << std::endl;
//         }
//     }
//     // for (; ite != TagetNumArray.end(); ite++){
//     //     std::cout << "array:" << *ite << std::endl;
//     // }*EP_Nav::ite
// }

geometry_msgs::Pose2D EP_Nav::ToPose(Posearray poseIn,int num)
{
    geometry_msgs::Pose2D p;
    p.x = poseIn.x[num];
    p.y = poseIn.y[num];
    p.theta = poseIn.th[num];
    return p;
}

void EP_Nav::run()
{
    //if(!TagetNumArray.empty() && TargetGetFlag)
    // if(true)
    // {      
        //std::cout << result <<std::endl;
        //std::cout << "array1:" << TagetNumArray[2] << std::endl;
    if (!DetectFlag)
    {
        if (newGoal)
        {
            GotoTarget(pose_targets, 6);
            newGoal = false;
        }
            
        Target = ToPose(pose_targets, 6);
        newGoal = false;
        // result = navCore->getMoveBaseActionResult();
        // std::cout << "out:" << Target << std::endl;
        // std::cout << "out2:" << pose_targets.x[6] << ";" << pose_targets.y[6] << std::endl;
        if(ArrivalGoal(Target))
        {
            serviceCaller->Detect_worker(DetectNum , uint32_t(9));
            TagetNumArray.push_back(serviceCaller->DetectTarget[0]);
            TagetNumArray.push_back(serviceCaller->DetectTarget[1]);
            TagetNumArray.push_back(serviceCaller->DetectTarget[2]);
            DetectFlag = true;
            newGoal = true;
            std::cout << "true" << TagetNumArray[2] << std::endl;
        }               
        else
        {
            DetectFlag = false;
            // std::cout << "flase" << ArrivalGoal(Target) << std::endl;
        }     
    }
    if(DetectFlag)
    {
        if(newGoal)
        {
            GotoTarget(pose_targets, TagetNumArray[NUMS]);
            newGoal = false;
        }
            
        Target = ToPose(pose_targets, TagetNumArray[NUMS]);            
        std::cout << "nums: " << NUMS << "; targetnum: "<< TagetNumArray[NUMS] << std::endl;
        std::cout << "out2: " << pose_targets.x[TagetNumArray[NUMS]] << "; " << pose_targets.y[TagetNumArray[NUMS]] << std::endl;
        if(ArrivalGoal(Target))
        {
            serviceCaller->Grasp_worker(Grasp , TagetNumArray[NUMS]);
            if(serviceCaller->GraspDone)
                PutFlag = true;
        }
        if(PutFlag)
        {
            GotoTarget(pose_targets, 0);
            Target = ToPose(pose_targets, 0); 
            if(ArrivalGoal(Target))
            {
                //serviceCaller->Put_worker(Put , TagetNumArray[NUMS]);
                //if(serviceCaller->PutDone)
                PutFlag = false;
                newGoal = true;
                if (NUMS < TagetNumArray.size())
                    NUMS++;
            }
            else
                newGoal = false;
        }
    }
    // result = navCore->getMoveBaseActionResult();
    // std::cout << "goal" << result << std::endl;
    // } 
    // for (; ite != TagetNumArray.end(); ite++){
    //     std::cout << "array:" << *ite << std::endl;
    // }
}


int main(int argc, char** argv)
{
    ros::init(argc , argv, "sim2real_client");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);
    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("map_frame",map_frame,(std::string)"map");
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS1");
    nh_.param("publish_tf",publish_tf,(bool)false);
    EP_Nav nav(base_foot_print,odom_frame,map_frame,serial_addr,publish_tf,nh_);
    nav.MAP_FRAME = map_frame;
    nav.BASE_FOOT_PRINT = base_foot_print;
    
    while (ros::ok())
    {
        nav.run();
        geometry_msgs::Pose2D currentpose;
        currentpose = nav.navCore->getCurrentPose(map_frame,base_foot_print);
        // std::cout << currentpose << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}