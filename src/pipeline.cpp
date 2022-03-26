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

//hello 
//test_error_kevin
class ServiceCaller{
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    ros::NodeHandle nh;
    ros::ServiceClient Detect_client;
    ros::ServiceClient Grasp_client;
    ros::ServiceClient Put_client;
    // std::tr1::shared_ptr<boost::thread> thread_ptr_;
    std::atomic_bool srvCalling_{},srvFinished_{};
public:
    ServiceCaller();
    ~ServiceCaller() = default;
    uint32_t DetectTarget[3];
    void Detect_worker(uint32_t flag,uint32_t num);
    void Grasp_worker(uint32_t flag, uint32_t NUM);
    void Put_worker(uint32_t flag, uint32_t NUM);
    bool DetecDone{false}, GraspDone{false},PutDone{false};
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
    geometry_msgs::Pose2D Target3point;
    bool TargetGetFlag{false},newGoal{true},DetectFlag{false},done{false},PutFlag{false},arrival{false};
    bool GraspFlag{false},ToPut{false},PutOnce{true},threeTransFlag{true},threePutFlag{true},GoOnce{true},GoPutOnce{true};
    ros::Publisher log_pub;
    ServiceCaller* serviceCaller;
    ros::Subscriber subTarget;
    ros::Publisher pub_move_cmd;
    ros::Publisher base_move_vel_pub;
    NavCore::MoveBaseActionResult result{NavCore::EMPTY};
    // void TargetCallback(const std_msgs::String::ConstPtr& msg_p);
    void TargetNumberCallback(const std_msgs::String::ConstPtr& msg_p);
    void setGoal(const geometry_msgs::Pose2D &goal2d);
    void Move_cmd(geometry_msgs::Twist poseIn ,geometry_msgs::Pose2D PoseTarget);
    void Move_cmd_Stop();
    bool ArrivalGoal(geometry_msgs::Pose2D PosrIn);
    bool Arrivalxy(geometry_msgs::Pose2D poseIn);
public:
    // EP_Nav(const std::string& base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf);
    // ~EP_Nav();
    enum Case 
    {
        DetectNum = 1,
        Grasp = 2,
        Put = 6,
    };
    
    struct Posearray{
        double x[9] , y[9] , th[9] ;
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
        done = false;
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
    Put_client = nh.serviceClient<s2r_pipeline::TargetNumber>("/detect_grasp_place_service");

    
    
    // client =nh.serviceClient<sim2real::sim2real_srv>("manipulate");
}
void ServiceCaller::Detect_worker(uint32_t flag ,uint32_t num)
{
    std::cout << "Detect worker " <<std::endl;
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

void ServiceCaller::Put_worker(uint32_t  flag , uint32_t NUM)
{
    std::cout << "Put_worker" <<std::endl;
    s2r_pipeline::TargetNumber Srv;
    Srv.request.number = NUM;
    Srv.request.work_case = flag;
    Put_client.call(Srv);
    PutDone = Srv.response.Detect_result;
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
    std::cout << "setgoal" << std::endl;
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
    //std::cout << "isarrival?" << std::endl;
    if(dx < 0.1 && dy < 0.1)
    {
        arrival = true;
    }
    else
        arrival = false;
    if(arrival)
    {
        std::cout << "arrive xy" << std::endl;
        if(!done)
        {
            navCore->cancelAllGoals();
            // currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);                
            // dth = currentpose.theta - PoseTarget.theta;
            // MoveTwist.angular.z = -dth;
            Move_cmd(MoveTwist,PoseTarget);
            std::cout << "move th xy done!" << std::endl;
            done = true;
            arrival =false;
            done = false;
            return true;
        }
        // if(std::abs(dth) < 0.1 )
        // {
        //     std::cout << "stop" << std::endl;
        //     Move_cmd_Stop();
        //     std::cout << "arrive th" << std::endl;
        //     arrival =false;
        //     done = false;
        //     return true;
        // }
        // else 
        // {
        //     std::cout << "unarrive th!!!" << std::endl;
        //     currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);                
        //     dth = currentpose.theta - PoseTarget.theta;
        //     MoveTwist.angular.z = -dth;
        //     Move_cmd(MoveTwist);
        // }
    }
    else 
        return false;
}

void EP_Nav::Move_cmd(geometry_msgs::Twist poseIn ,geometry_msgs::Pose2D PoseTarget)
{
    geometry_msgs::Twist pose;     
    geometry_msgs::Pose2D currentpose;
    double dx,dy,dth,Cth,Tth ,turn,midx,midy,speed;
    midx = 0.336;
    midy = 1.2; 
    speed = 0.11;      
    currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);                
    dth = currentpose.theta - PoseTarget.theta;
    Cth = currentpose.theta;
    Tth = PoseTarget.theta;

    while(std::abs(dth) > 0.02)
    {
        if (Cth * Tth < 0)
        {
            if ( abs(Cth)+abs(Tth) > 3.14)
            {
                if(Cth > 0)
                    turn = 1.0;
                else
                    turn = -1.0;
            }
            else 
            {
                if(Cth > 0)
                    turn = -1.0;
                else
                    turn = 1.0;
            }
        }
        else if(Cth < 0)
            turn = (Tth - Cth)>0 ? 1.0 : -1.0;
        else
            turn = (Tth -Cth)>0 ? 1.0:-1.0;
        pose.linear.x = 0;
        pose.linear.y = 0;
        pose.linear.z = 0;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = turn;
        base_move_vel_pub.publish(pose);
        currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);                
        dth = currentpose.theta - PoseTarget.theta;
    }
    Move_cmd_Stop();
    currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);  
    dx = PoseTarget.x - currentpose.x  ;
    dy = PoseTarget.y - currentpose.y  ; // dy>0 left
    while(abs(dx) > 0.05)
    {
        currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);  
        dx = PoseTarget.x - currentpose.x ;
        std::cout << "dx = : " << dx << std::endl;
        if(PoseTarget.theta < -3)
        {
            if(currentpose.x + midx < PoseTarget.x + midx) 
                dx = -speed;
            else
                dx = speed;
        }
        else if(PoseTarget.theta < 0)
        {
            if(currentpose.y + midy < PoseTarget.y + midy)
                dx = -speed;
            else 
                dx = speed;
        }
        else
        {
            if(currentpose.x + midx < PoseTarget.x + midx)
                dx = speed;
            else 
                dx = -speed;
        }

        dy = PoseTarget.y - currentpose.y  ; // dy>0 left
        std::cout << "dy = : " << dy << std::endl;
        if(PoseTarget.theta < -3)
        {
            if(currentpose.y + midy < PoseTarget.y + midy)
                dy = speed;
            else
                dy = -speed;
        }
        else if(PoseTarget.theta < 0)
        {
            if(currentpose.x + midx < PoseTarget.x + midx)
                dy = speed;
            else
                dy = -speed;
        }
        else
        {
            if(currentpose.y + midy < PoseTarget.y + midy)
                dy = speed;
            else
                dy = -speed;
        }
        pose.linear.x = dx;
        pose.linear.y = dy;
        pose.linear.z = 0;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = 0;
        base_move_vel_pub.publish(pose);     
        currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);  
        dx = PoseTarget.x - currentpose.x ;   
        dy = PoseTarget.y - currentpose.y ;  
    }
    Move_cmd_Stop();
    // while(abs(dy) > 0.09)
    // {
    //     currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);  
    //     // dy = PoseTarget.y - currentpose.y  ; // dy>0 left
    //     // std::cout << "dy = : " << dy << std::endl;
    //     // if(PoseTarget.theta < -3)
    //     // {
    //     //     if(currentpose.y + midy < PoseTarget.y + midy)
    //     //         dy = -speed;
    //     //     else
    //     //         dy = speed;
    //     // }
    //     // else if(PoseTarget.theta < 0)
    //     // {
    //     //     if(currentpose.x + midx < PoseTarget.x + midx)
    //     //         dy = speed;
    //     //     else
    //     //         dy = -speed;
    //     // }
    //     // else
    //     // {
    //     //     if(currentpose.y + midy < PoseTarget.y + midy)
    //     //         dy = speed;
    //     //     else
    //     //         dy = -speed;
    //     // }
    //     pose.linear.x = 0;
    //     pose.linear.y = dy;
    //     pose.linear.z = 0;
    //     pose.angular.x = 0;
    //     pose.angular.y = 0;
    //     pose.angular.z = 0;
    //     base_move_vel_pub.publish(pose);   
    //     currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);  
    //     dy = PoseTarget.y - currentpose.y ;   
    // }
    // Move_cmd_Stop();
    // sleep(0.5);
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
    posearray.x[0] = 1.01 ,posearray.y[0] = 1.73, posearray.th[0] = 0.00;    //Box
    posearray.x[1] = 0.633 ,posearray.y[1] = 3.18, posearray.th[1] = -3.13;    //num1
    posearray.x[2] = 0.384 ,posearray.y[2] = 2.9, posearray.th[2] = -1.57;    //num2
    // posearray.x[3] = 2.504 ,posearray.y[3] = 2.441, posearray.th[3] = 1.599;    //num3
    posearray.x[3] = 2.0 ,posearray.y[3] = 2.68, posearray.th[3] = 0.0;    //num3
    posearray.x[4] = 2.22 ,posearray.y[4] = 0.014, posearray.th[4] = -3.13;    //num4
    posearray.x[5] = 2.69 ,posearray.y[5] = -0.859, posearray.th[5] = 0.00;   //num5
    posearray.x[6] = 0.167 ,posearray.y[6] = 1.512, posearray.th[6] = 0.00;  //detect GoalNums
    posearray.x[7] = 1.25 ,posearray.y[7] = 2.61, posearray.th[7] = 0;
    posearray.x[8] = 1.17 ,posearray.y[8] = 0.977, posearray.th[8] = 0;
     
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

bool EP_Nav::Arrivalxy(geometry_msgs::Pose2D poseIn)
{
    geometry_msgs::Pose2D currentpose;
    double dx,dy,dth;
    currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);
    dx = abs(currentpose.x - Target3point.x);
    dy = abs(currentpose.y - Target3point.y);  
    if(dx < 0.15 && dy < 0.15)
    {   
        return true;
    }
    else
        return false;
}

void EP_Nav::run()
{
    result = navCore->getMoveBaseActionResult();
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
            // if(TagetNumArray[NUMS] == 3 || TagetNumArray[NUMS] == 1 || TagetNumArray[NUMS] == 2 && GoOnce)
            // {
            //     GoOnce = false;
            //     threeTransFlag = false;
            //     GotoTarget(pose_targets, 7);
            //     Target3point = ToPose(pose_targets, 7); 
            // }
            // if (!GoOnce)
            // {
            //     if(Arrivalxy(Target3point))
            //     {
            //         threeTransFlag = true;                    
            //     }
            // }
            threeTransFlag = true;             
            if(threeTransFlag)
            {
                GotoTarget(pose_targets, TagetNumArray[NUMS]);
                newGoal = false;
                
                std::cout << "nums: " << NUMS << "; targetnum: "<< TagetNumArray[NUMS] << std::endl;
                GraspFlag = true;
            }
        }
        if(GraspFlag)
        {
            Target = ToPose(pose_targets, TagetNumArray[NUMS]);            
             //std::cout << "out2: " << pose_targets.x[TagetNumArray[NUMS]] << "; " << pose_targets.y[TagetNumArray[NUMS]] << std::endl;
            if(ArrivalGoal(Target))
            {
                serviceCaller->Grasp_worker(Grasp , TagetNumArray[NUMS]); 
                GraspFlag = false;
            }                                    
        }
        if(serviceCaller->GraspDone && !PutFlag)
        {
            PutFlag = true;
            ToPut = true;
            std::cout << "PutFlas:true"  << std::endl;
            serviceCaller->GraspDone = false;
        }          
        if(PutFlag)
        {
            // if((TagetNumArray[NUMS] == 3 || TagetNumArray[NUMS] == 1 || TagetNumArray[NUMS] == 2) && GoPutOnce)
            // {
            //     std::cout<<"put3back"<<std::endl;
            //     GoPutOnce = false;
            //     threePutFlag = false;
            //     GotoTarget(pose_targets, 7);
            //     Target3point = ToPose(pose_targets, 7 ); 
            // }
            // if (!GoPutOnce)
            // {
            //     std::cout<<"judge 3 put back"<<std::endl;
            //     if(Arrivalxy(Target3point))
            //     {
            //         std::cout<<"arrive 3 back"<<std::endl;
            //         threePutFlag = true;                    
            //     }
            // }   
            //navCore->cancelAllGoals();
            threePutFlag = true; 
            if (ToPut && threePutFlag)
            {
    
                GotoTarget(pose_targets,0);
                ToPut = false;
                std::cout << "gotozero" << std::endl;
            }
            Target = ToPose(pose_targets,0); 
            if(ArrivalGoal(Target))
            {
                if(PutOnce)
                {
                    serviceCaller->Put_worker(Put , NUMS);
                    PutOnce = false;
                    std::cout << "putcall" << std::endl;
                }

                if(serviceCaller->PutDone)
                {
                    serviceCaller->PutDone = false;
                    std::cout << "putdone:next goal"  << std::endl;
                    PutFlag = false;
                    newGoal = true;
                    GoOnce = true;
                    PutOnce = true;
                    GoPutOnce = true;
                    if (NUMS < TagetNumArray.size())
                    {
                        NUMS++;
                        Target = ToPose(pose_targets,TagetNumArray[NUMS]);
                    }
                        
                }
            }
            else
                newGoal = false;
        }
    }

}


int main(int argc, char** argv)
{
    sleep(3);
    ros::init(argc , argv, "sim2real_client");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(30);
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