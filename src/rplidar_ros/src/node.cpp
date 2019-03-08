#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "rplidar.h"
#include "std_msgs/String.h"
#include <rplidar_ros/coordinate_msg.h>

//OpenCV2标准头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <queue>
#define PI acos(-1)
using namespace rp::standalone::rplidar;
using namespace std;
using namespace cv;
int ksize = 5; //5         // ksize个点拟合一条直线
double deltaAngMax = 20; // 小于deltaAngMax认为可能是一条直线
int deltaPointNum = 20;
std::queue<double> selectData;
std::queue<double> selectAngle;

double angle_memory=1000;
int angle_memory_len=0;
int angle_memory_index=0;

struct Result
{
    double x;
    double y;
    double ang;
};

Result calPosion(std::vector<std::vector<double>> &nodes, int count);
cv::Mat polyfit(std::vector<cv::Point2d> &in_point, int n);
double distance(cv::Point2d p1, cv::Point2d p2);
bool dataSelect(std::queue<double>  selectData,double x);
bool angleSelect(std::queue<double>  selectData,double x);
double hough(std::vector<std::vector<double>>& data, int count);
// int deltaAngle(int a,int b);
double deltaAngle(double a,double b){
    if(a==b) return 0;
    double c=max(a,b);
    double d=min(a,b);
    return min(c-d,d-c+360);
}
void fit_line(std::vector<double> &kb,std::vector<cv::Point2d> &tempVec);

double dis_cos(double ang1,double r1,double ang2,double r2){
    return sqrt(r1*r1+r2*r2-2*r1*r2*cos(ang1-ang2));
}
double angle2Radian(double angle){
    return angle/180*PI;
}
double radian2Angle(double radian){
    return radian*180/PI;
}

double OFF_SET=61.7334;                                                                                                                                                                                                                                                                
void polyContourFit(std::vector<cv::Point2d> &XYdata,std::vector<int> &breakPoint,int start, double eps){
    int n=XYdata.size();
    if(n<=2){
        return;
    }
    double dis=distance(XYdata[0],XYdata[n-1]);
    double cosTheta = (XYdata[n-1].x - XYdata[0].x) / dis;
    double sinTheta = - ( XYdata[n-1].y - XYdata[0].y )/dis;
    double MaxDis = 0;
    int i ;
    int maxDisInd = -1;
    double dbDis;
    for(i = 1 ; i < n - 1 ; i++)
    {
        // 进行坐标旋转，求旋转后的点到x轴的距离
        dbDis = abs( (XYdata[i].y - XYdata[0].y) * cosTheta
        + (XYdata[i].x - XYdata[0].x)* sinTheta);
        if( dbDis > MaxDis)
        {
            MaxDis = dbDis;
            MaxDis = dbDis;
            maxDisInd = i;
        }
    }
    if(MaxDis > eps && (maxDisInd>2 && n-1-maxDisInd>2))
    {
        std::vector<cv::Point2d> XYdataTemp;
        copy(XYdata.begin(), XYdata.begin() + maxDisInd, std::back_inserter(XYdataTemp));
        polyContourFit(XYdataTemp,breakPoint,start,100);

        breakPoint.push_back(start+maxDisInd);

        XYdataTemp.clear();
        copy(XYdata.begin() + maxDisInd, XYdata.end(), std::back_inserter(XYdataTemp));
        polyContourFit(XYdataTemp,breakPoint,start+maxDisInd,100);
        return;
    }else
    {
        return;
    }


}
double angleLength(std::vector<cv::Point2d> &XYdata,int a,int b,int c){
    return distance(XYdata[a],XYdata[b])+distance(XYdata[b],XYdata[c]);
}
double directAngle(std::vector<cv::Point2d> &XYdata,int a,int b,int c){
    double x1=XYdata[a].x-XYdata[b].x;
    double y1=XYdata[a].y-XYdata[b].y;
    double x2=XYdata[c].x-XYdata[b].x;
    double y2=XYdata[c].y-XYdata[b].y;
    return x1*y2-x2*y1;
}
bool isVertical(std::vector<cv::Point2d> &XYdata,int a,int b,int c){
    // double x1=XYdata[a].x-XYdata[b].x;
    // double y1=XYdata[a].y-XYdata[b].y;
    // double x2=XYdata[c].x-XYdata[b].x;
    // double y2=XYdata[c].y-XYdata[b].y;
    double area=abs(directAngle(XYdata,a,b,c));
    double l1=distance(XYdata[a],XYdata[b]);
    double l2=distance(XYdata[b],XYdata[c]);
    if(l1*l2/area>1.414){
        return false;
    }else
    {
        return true;
    }

}
bool needFlip(std::vector<cv::Point2d> &XYdata,int a,int b){
    return abs(XYdata[a].x-XYdata[b].x)<abs(XYdata[a].y-XYdata[b].y);
}
bool needFlip(std::vector<cv::Point2d> &XYdata){
    int n=XYdata.size();
    return abs(XYdata[0].x-XYdata[n-1].x)<abs(XYdata[0].y-XYdata[n-1].y);
}
vector<double> resolveKBAngle(std::vector<cv::Point2d> &XYdata){
    double ang=0;
    double r=0;
    double k,b;
    if(needFlip(XYdata)){
        std::vector<cv::Point2d> YXdata(XYdata.size());
        for(int i = 0; i < XYdata.size(); i++)
        {
            YXdata[i].x=XYdata[i].y;
            YXdata[i].y=XYdata[i].x;
        }
        cv::Mat Mat_k = polyfit(YXdata, 1);
        b = Mat_k.at<double>(0, 0);
        k = Mat_k.at<double>(1, 0);
        r=abs(b)/sqrt(k*k+1);
        if (abs(k)<0.00001) {
            k=k>0?100000:-100000;
        }else
        {
            k=1.0/k;
        }
        ang=atan(k)*180/PI;
        b=-k*b;

    }else
    {
        cv::Mat Mat_k = polyfit(XYdata, 1);
        b = Mat_k.at<double>(0, 0);
        k = Mat_k.at<double>(1, 0);
        r=abs(b)/sqrt(k*k+1);
        ang=atan(k)*180/PI;
    }
    vector<double> ans;
    ans.push_back(ang);
    ans.push_back(r);
    ans.push_back(k);
    ans.push_back(b);
    return ans;

}


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI / 180.)


RPlidarDriver *drv = NULL;
// cv::Mat m=NULL;
// void publish_scan(ros::Publisher *pub,
//                   rplidar_response_measurement_node_hq_t *nodes,
//                   size_t node_count, //ros::Time start,
//                   //double scan_time,
//                   bool inverted,
//                   float angle_min, float angle_max,
//                   float max_distance,
//                   //   std::string frame_id
// )
// {
//     std::vector<std::vector<double>> data(800, std::vector<double>(2));
//     // static int scan_count = 0;
//     // sensor_msgs::LaserScan scan_msg;

//     // scan_msg.header.stamp = start;
//     // scan_msg.header.frame_id = frame_id;
//     // scan_count++;
//     float real_angle_min, real_angle_max;

//     bool reversed = (angle_max > angle_min);
//     if (reversed)
//     {
//         real_angle_min = M_PI - angle_max;
//         real_angle_max = M_PI - angle_min;
//     }
//     else
//     {
//         real_angle_min = M_PI - angle_min;
//         real_angle_max = M_PI - angle_max;
//     }
//     float real_angle_increment =
//         (real_angle_max - real_angle_min) / (double)(node_count - 1);

//     // scan_msg.scan_time = scan_time;
//     // scan_msg.time_increment = scan_time / (double)(node_count-1);
//     // scan_msg.range_min = 0.15;
//     // scan_msg.range_max = max_distance;//8.0;

//     // scan_msg.intensities.resize(node_count);
//     // scan_msg.ranges.resize(node_count);
//     for (int i = 0; i < count; i++)
//     {
//         // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
//         if (scan->ranges[i] * 1000 < 4600)
//         {
//             float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
//             data[len][0] = degree;
//             data[len][1] = scan->ranges[i] * 1000;
//             ++len;
//         }
//     }

//     bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
//     int len = 0;
//     if (!reverse_data)
//     {
//         for (size_t i = 0; i < node_count; i++)
//         {
//             // float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
//             // if (read_value == 0.0)
//             //     scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
//             // else
//             //     scan_msg.ranges[i] = read_value;
//             // scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);

//             float read_value = (float)nodes[i].dist_mm_q2 / 4.0f;
//             if (read_value > 4600 || read_value == 0.0)
//             {
//             }
//             else
//             {
//                 data[len][0] = RAD2DEG(real_angle_min + real_angle_increment * i);
//                 data[len][1] = read_value;
//                 ++len;
//             }
//         }
//     }
//     else
//     {
//         for (size_t i = 0; i < node_count; i++)
//         {

//             float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
//             if (read_value == 0.0)
//                 scan_msg.ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
//             else
//                 scan_msg.ranges[node_count - 1 - i] = read_value;
//             scan_msg.intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
//         }
//     }

//     pub->publish(scan_msg);
// }

bool getRPLIDARDeviceInfo(RPlidarDriver *drv)
{
    u_result op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result))
    {
        if (op_result == RESULT_OPERATION_TIMEOUT)
        {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        }
        else
        {
            ROS_ERROR("Error, unexpected error, code: %x", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    ROS_INFO("Firmware Ver: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d", (int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver *drv)
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result))
    {
        ROS_INFO("RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}

bool stop_motor(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res)
{
    if (!drv)
        return false;

    ROS_DEBUG("Stop motor");
    drv->stop();
    drv->stopMotor();
    return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res)
{
    if (!drv)
        return false;
    ROS_DEBUG("Start motor");
    drv->startMotor();
    drv->startScan(0, 1);
    return true;
}

static float getAngle(const rplidar_response_measurement_node_hq_t &node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    // ROS_INFO("node heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char *argv[])
{
    std::vector<std::vector<double>> data(800, std::vector<double>(2));
    
    rplidar_ros::coordinate_msg msg;

    ros::init(argc, argv, "rplidar_node");

    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    int angle_compensate_multiple = 1; //it stand of angle compensate at per 1 degree
    std::string scan_mode;
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/serial/read", 1, chatterCallback);
    ros::Publisher scan_pub = nh.advertise<rplidar_ros::coordinate_msg>("scan", 1);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200 /*256000*/); //ros run for A1 A2, change to 256000 if A3
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, false);
    nh_private.param<bool>("angle_compensate", angle_compensate, false);
    nh_private.param<std::string>("scan_mode", scan_mode, std::string());

    ROS_INFO("RPLIDAR running on ROS package rplidar_ros. SDK Version:");

    u_result op_result;

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

    if (!drv)
    {
        ROS_ERROR("Create Driver fail, exit");
        return -2;
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate)))
    {
        ROS_ERROR("Error, cannot bind to the specified serial port %s.", serial_port.c_str());
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv))
    {
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv))
    {
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
    ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

    drv->startMotor();

    RplidarScanMode current_scan_mode;
    if (scan_mode.empty())
    {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }
    else
    {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result))
        {
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter =
                     allSupportedScanModes.begin();
                 iter != allSupportedScanModes.end(); iter++)
            {
                if (iter->scan_mode == scan_mode)
                {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1))
            {
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:",
                          scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin();
                     iter != allSupportedScanModes.end(); iter++)
                {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
                              iter->max_distance, (1000 / iter->us_per_sample));
                }
                op_result = RESULT_OPERATION_FAIL;
            }
            else
            {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }
    _u16 pwm = 320;//300时大概800个点
    drv->setMotorPWM(pwm);

    if (IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple = (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
        if (angle_compensate_multiple < 1)
            angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        ROS_INFO("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", current_scan_mode.scan_mode,
                 current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample), angle_compensate_multiple);
    }
    else
    {
        ROS_ERROR("Can not start scan: %08x!", op_result);
    }

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    while (ros::ok())
    {
        rplidar_response_measurement_node_hq_t nodes[360 * 8];
        size_t count = _countof(nodes);

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec();

        if (op_result == RESULT_OK)
        {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            if (op_result == RESULT_OK || op_result == RESULT_OPERATION_FAIL)
            {
                //
                clock_t t1;
                t1 = clock();
                int len = 0;
                for (int i = 0; i < count; i++)
                {
                    // ROS_INFO(": [point:%05.1lf  %05.1lf]", getAngle(nodes[i]),(float)nodes[i].dist_mm_q2 / 4.0f);
                    if ((getAngle(nodes[i])>132 && getAngle(nodes[i])<210) && (float)nodes[i].dist_mm_q2 / 4.0f < 800) {
                        continue;
                    }
                    
                    if ((float)nodes[i].dist_mm_q2 / 4.0f < 4600)
                    {
                        data[len][0] = getAngle(nodes[i]);
                        data[len][1] = (float)nodes[i].dist_mm_q2 / 4.0f;
                        ++len;
                    }
                }
                // ROS_INFO(": [allNode count:%d  ]", count);
                len=len-1;
                Result r;
                r = calPosion(data, len);
                ROS_INFO(": [node:%05.1lf  %05.1lf  %03.1lf]", r.x, r.y, r.ang);
                if(r.x>50&& r.y>50){
                    
                    
                    if(deltaAngle((int)(angle_memory+180),(int)(r.ang+180))>15){
                        angle_memory = r.ang;
                        continue;
                    }
                    angle_memory = r.ang;



                    // double hough_a=hough(data,len);
                    // if(hough_a>-180.1){
                    //     ROS_INFO("[hough:%03.1lf ]",hough_a);
                    //     if (!(std::abs(r.ang - hough_a) < 20 
                    //         || std::abs(std::abs(r.ang) + std::abs(hough_a) - 2 * 180) < 20
                    //         || std::abs(r.ang - (180-hough_a)) < 20 
                    //         || std::abs(std::abs(r.ang) + std::abs(180-hough_a) - 2 * 180) < 20)){
                    //             continue;
                    //         }else{
                    //             angle_memory=r.ang;

                    //         }
                    // }
                    // else if (std::abs(r.ang - angle_memory) < 20 || std::abs(std::abs(r.ang) + std::abs(angle_memory) - 2 * 180) < 20)
                    // {
                    //     angle_memory = r.ang;
                    // }else{
                    //     continue;
                    // }
                }
                


                

                std_msgs::String result;
                // result.data = ser.read(ser.available());
                // getData(result.data, horizonal, vertical);

                //TODO 如果需要发布数据，这个地方可能需要修改，因为跳过了ros::spinOnce();
                if (r.x < 0)
                {
                    double angle_increment;
                    for (angle_increment = -45; angle_increment < 45; angle_increment += 5)
                    {
                        // double angle_increment=-45;
                        for (int i = 0; i < len; i++)
                        {
                            data[i][0] += angle_increment;
                        }
                        r = calPosion(data, len);
                        if (r.x > 0)
                        {
                            r.ang -= angle_increment;
                            if (r.ang <= -180)
                            {
                                r.ang += 360;
                            }
                            else if (r.ang >= 180)
                            {
                                r.ang -= 360;
                            }
                            r.ang=angle_memory;
                            break;

                        }
                    }
                    if(r.x<50)
                    {
                        continue;
                        msg.coordinate_x = 6000;
                        msg.coordinate_y = 6000;
                        msg.angle = 200;
                    }
                    
                }

                if(r.x<30 || r.y<30){
                    // r.ang+=90;
                    double angle=r.ang;
                    double angle_increment=45+r.ang;
                    
                    for(int i = 0; i < len; i++)
                    {
                        data[i][0]+=angle_increment;
                    }
                    r = calPosion(data, len);
                    if(r.x>0 && r.y>0){
                        r.ang=45-angle_increment;
                        if(r.ang<=-180){
                            r.ang+=360;
                        }else if(r.ang>=180){
                            r.ang-=360;
                        }
                        if (angle_memory < 181 && angle_memory > -181)
                        {
                            if (!(std::abs(r.ang - angle_memory) < 20 
                            || std::abs(std::abs(r.ang) + std::abs(angle_memory) - 2 * 180) < 20))
                            {
                                r.ang = angle_memory;
                                
                            }else{
                                angle_memory=r.ang;
                            }
                        }else{
                            continue;
                        }
                    }
                }
                if (r.x <= 30|| r.y<=30)
                {
                    continue;
                    msg.coordinate_x = 6000;
                    msg.coordinate_y = 6000;
                    msg.angle = 200;
                }

                if(selectData.size()<7){
                    selectData.push(r.x);
                    // selectAngle.push(r.ang);
                    // angle_memory[angle_memory_index++]=r.ang;
                    continue;
                }else{
                    if(dataSelect(selectData,r.x) ){//&& angleSelect(selectAngle,r.ang)
                        msg.coordinate_x = r.x;
                        msg.coordinate_y = r.y;
                        msg.angle = r.ang;

                        scan_pub.publish(msg);
                    }

                    
                    selectData.pop();
                    // selectAngle.pop();                    
                    selectData.push(r.x);
                    // selectAngle.push(r.ang);
                }
                ros::spinOnce();
            }
            // else if (op_result == RESULT_OPERATION_FAIL)
            // {

            // }
        }

        ros::spinOnce();
    }

    // done!
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

Result calPosion(std::vector<std::vector<double> > &nodes, int count)
{
    vector<vector<int> > region;
    if (count < 20)
    {
        struct Result r;
        r.y = -1001;
        r.x = -1001;
        r.ang = 400;
        return r;
    }
    try
    {
        std::vector<cv::Point2d> XYdata(count);
        //std::vector<std::vector <double> > data(800, std::vector<double>(2));
        for (int pos = 0; pos < count; ++pos)
        {
            XYdata[pos].x = nodes[pos][1] * cos(nodes[pos][0] * PI / 180);
            XYdata[pos].y = nodes[pos][1] * sin(nodes[pos][0] * PI / 180);
            /*XYdata[pos][0] = nodes[pos][1] * cos(nodes[pos][0]);
			XYdata[pos][1] = nodes[pos][1] * sin(nodes[pos][0]);*/
        }

        int i=0;
        while(i<count){
            vector<int> col;
            col.push_back(i);
            int j=i+1;
            while(j<count){
//            cout<<dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[j-1][0]),nodes[j-1][1])<<" "
//                <<15*(nodes[j][1]+nodes[i][1])/2*angle2Radian(deltaAngle(nodes[j][0],nodes[i][0]))//15*(nodes[j][1]+nodes[j-1][1])/2*abs(angle2Radian(nodes[j][0])-angle2Radian(nodes[j-1][0]))<<" "
//                << dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[j-1][0]),nodes[j-1][1])<<" "
//                << abs(nodes[j][0]-nodes[j-1][0])<<endl;

                if(( (dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[j-1][0]),nodes[j-1][1])
                <18*(nodes[j][1]+nodes[j-1][1])/2*angle2Radian(deltaAngle(nodes[j][0],nodes[j-1][0])))//15*(nodes[j][1]+nodes[j-1][1])/2*abs(angle2Radian(nodes[j][0])-angle2Radian(nodes[j-1][0]))
                || dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[j-1][0]),nodes[j-1][1])<50)
                && deltaAngle(nodes[j][0],nodes[j-1][0])<10 ){
                    col.push_back(j);
                    j+=1;
                }else
                {
                    break;
                }
            }
            region.push_back(col);
            i=j;
        }
        i=count-1;
        int j=(i+1)%count;
        if(( (dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[i][0]),nodes[i][1])
        <18*(nodes[j][1]+nodes[i][1])/2*angle2Radian(deltaAngle(nodes[j][0],nodes[i][0])))//2*abs(angle2Radian(nodes[j][0])-angle2Radian(nodes[i][0]))//此处0度减去360度的问题
        || dis_cos(angle2Radian(nodes[j][0]),nodes[j][1],angle2Radian(nodes[i][0]),nodes[i][1])<50)
        && deltaAngle(nodes[j][0],nodes[i][0])<10 ){
            region[region.size() - 1].insert(region[region.size() - 1].end(), region[0].begin(), region[0].end());
            region.erase(region.begin());
        }

        for (int i = region.size() - 1; i >= 0; i--){
            if (region[i].size() <= 2){
                region.erase(region.begin() + i);
            }
        }
//        cout<<"region.size()"<<region.size()<<endl;
//        for(int i=0;i<region.size();i++){
//            for(int j=0;j<region[i].size();j++){
//                cout<<region[i][j]<<" ";
//            }
//            cout<<endl;
//        }
        vector<vector<double> > ans;
        // 基于每个区域进行查找
        for(int i = 0; i < region.size(); i++)
        {
            vector<int>& col=region[i];
            std::vector<cv::Point2d> everyXYdata;
            for(int i = 0; i < col.size(); i++)
            {
                everyXYdata.push_back(XYdata[col[i]]);
            }
            std::vector<int> breakPoint;
            polyContourFit(everyXYdata,breakPoint,0,100);
//            sort(breakPoint.begin(), breakPoint.end());

//            for(int i=0;i<breakPoint.size();i++){
//                cout<<breakPoint[i]<<endl;
//            }
//            cout<<endl;
            if(breakPoint.size()==0){
                continue;
            }
            vector<int> p;
            p.push_back(col[0]);
            for(int i = 0; i < breakPoint.size(); i++)
            {
                p.push_back(col[breakPoint[i]]);
            }
            p.push_back(col[col.size()-1]);

            for(int i = 1; i < p.size()-1; i++)
            {
                int a1=p[i-1];
                int a2=p[i];
                int a3=p[i+1];
                if(directAngle(XYdata,a1,a2,a3)<0 && angleLength(XYdata,a1,a2,a3)>500
                && isVertical(XYdata,a1,a2,a3) ){
                    double ang;
                    vector<cv::Point2d> l1,l2;
                    vector<double> ar1,ar2;
                    if(a1>a2){
                        copy(XYdata.begin()+a1, XYdata.end(), std::back_inserter(l1));
                        copy(XYdata.begin(), XYdata.begin()+a2, std::back_inserter(l1));
                    }else
                    {
                        copy(XYdata.begin()+a1, XYdata.begin()+a2, std::back_inserter(l1));
                    }
                    ar1=resolveKBAngle(l1);
                    if(a2>a3){
                        copy(XYdata.begin()+a2, XYdata.end(), std::back_inserter(l2));
                        copy(XYdata.begin(), XYdata.begin()+a3, std::back_inserter(l2));
                    }else
                    {
                        copy(XYdata.begin()+a2, XYdata.begin()+a3, std::back_inserter(l2));
                    }
                    ar2=resolveKBAngle(l2);
                    if(abs( abs(ar1[0]-ar2[0])-90 )<15 ){
                        if ( (XYdata[a1].x-XYdata[a2].x)*1+(XYdata[a1].y-XYdata[a2].y)*tan(ar1[0]/180*PI)<0 ) {
                            ang=ar1[0]>0?ar1[0]-180:ar1[0]+180;
                        }else
                        {
                            ang=ar1[0];
                        }

                        vector<double> v;
//                        double k1=ar1[2];
//            double b1=ar1[3];
//            double k2=ar1[2];
//            double b2=ar1[3];
//            if(std::abs(k1)>1000 || std::abs(k2)>1000){
//                v.push_back(ar2[1]);
//            }else{
//                double x_cood=(b2-b1)/(k1-k2);
//                double y_cood;
//                if(std::abs(k1)>std::abs(k2)){
//                    y_cood=k2*x_cood+b2;
//                }else{
//                    y_cood=k1*x_cood+b1;
//                }
//                v.push_back(sqrt(x_cood*x_cood+y_cood*y_cood-ar1[1]*ar1[1]));
//            }

                        v.push_back(ar2[1]);
                        v.push_back(ar1[1]);
                        v.push_back(ang);
                        v.push_back(distance(XYdata[a1],XYdata[a2]));
                        v.push_back(distance(XYdata[a2],XYdata[a3]));
                        ans.push_back(v);
                    }
                }
            }
        }
        if (ans.size()==0) {
            struct Result r;
            r.y = -1001;
            r.x = -1001;
            r.ang = 400;
            return r;
        }else
        {
            int MaxDisInd = -1;
            double MaxDis = 0;
            double dbDis;
            for(int i = 0; i < ans.size(); i++)
            {
                dbDis=ans[i][3]+ans[i][4];
                if(dbDis>MaxDis){
                    MaxDis=dbDis;
                    MaxDisInd=i;
                }
            }
            struct Result r;
            r.x = ans[MaxDisInd][0];
            r.y = ans[MaxDisInd][1];
            r.ang = -ans[MaxDisInd][2];
            r.ang-=61.7334;
            if(r.ang<-180){
                r.ang+=360;
            }
            return r;
        }
    }
    catch (...)
    {
        struct Result r;
        r.y = -11111111111;
        r.x = -11111111111;
        r.ang = -11111111111;
        return r;
    }
}

cv::Mat polyfit(std::vector<cv::Point2d> &in_point, int n)
{
    int size = in_point.size();
    //所求未知数个数
    int x_num = n + 1;
    //构造矩阵U和Y
    cv::Mat Mat_u(size, x_num, CV_64F);
    cv::Mat Mat_y(size, 1, CV_64F);

    for (int i = 0; i < Mat_u.rows; ++i)
        for (int j = 0; j < Mat_u.cols; ++j)
        {
            Mat_u.at<double>(i, j) = pow(in_point[i].x, j);
        }

    for (int i = 0; i < Mat_y.rows; ++i)
    {
        Mat_y.at<double>(i, 0) = in_point[i].y;
    }

    //矩阵运算，获得系数矩阵K
    cv::Mat Mat_k(x_num, 1, CV_64F);
    Mat_k = (Mat_u.t() * Mat_u).inv() * Mat_u.t() * Mat_y;
    //cout << Mat_k << endl;
    return Mat_k;
}
double distance(cv::Point2d p1, cv::Point2d p2)
{
    return sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
bool dataSelect(std::queue<double>  selectData,double x){
    double temp[7];
    double sum=0;
    double sum2=0;
    double d;
    for(int i=0;i<7;i++){
        sum+=selectData.front();
        temp[i]=selectData.front();
        selectData.pop();
    }
    sum/=7;
    for(int i=0;i<7;i++){
        sum2+=(temp[i]-sum)*(temp[i]-sum);
    }
    sum2=std::sqrt(sum2/7);
    if(x<sum-2*sum2 || x>sum+2*sum2){
        return false;
    }else{
        return true;
    }
}
// 这个方法有问题，因为当运行时很可能是线性变化，所以会误删，只能做静止时
bool angleSelect(std::queue<double>  selectData,double x){
    double temp[7];
    
    for(int i=0;i<7;i++){
        temp[i]=(int)(selectData.front()+180);
    }
    int a=(int)(x+180);

    int sum=0;
    for(int i=0;i<7;i++){
        //相差是否大于10度
        if(deltaAngle(a,temp[7])<20){
            sum++;
        }      
    }
    if(sum<=3){
        return false;
    }else{
        return true;
    }
}
int deltaAngle(int a,int b){
    return std::min(std::abs((a+360-b)%360),std::abs((b+360-a)%360));
}
void fit_line(std::vector<double> &kb,std::vector<cv::Point2d> &tempVec){
    cv::Mat Mat_k = polyfit(tempVec, 1);
    double b = Mat_k.at<double>(0, 0);
    double k = Mat_k.at<double>(1, 0);
    kb[0] = atan(k) * 180 / PI;
    //pList[i / ksize] = k;
    kb[2]=k;
    kb[3]=b;

    if(std::abs(k)>2){
        std::vector<cv::Point2d> reverseVec(tempVec.size());
        for(int i=0;i<tempVec.size();i++){
            reverseVec[i].x=tempVec[i].y;
            reverseVec[i].y=tempVec[i].x;
        }
        cv::Mat Mat_k2 = polyfit(reverseVec, 1);
        b = Mat_k2.at<double>(0, 0);
        k = Mat_k2.at<double>(1, 0);
        kb[1] = std::abs(b) / sqrt(k * k + 1);
    }else{
        kb[1] = std::abs(b) / sqrt(k * k + 1);
    }
}

double hough(std::vector<std::vector<double>>& data, int count){
    cv::Mat black_img = cv::Mat::zeros(cv::Size(2*4600, 2*4600), CV_8UC1);//

    // cout<<len<<endl;
	for(int i=0;i<count;i++){
        double theta=data[i][0]*PI/180;
        double r=data[i][1];
        int x=(int)(r*cos(theta)+4600);
        int y=(int)(r*sin(theta)+4600);
        black_img.at<uchar>(x,y)=255;
        // cout<<x<<" "<<y<<endl;   
//        if(x>=0 && x<9000)
//        if(y>=0 && y<9000){
//
//        }
	}
    //【3】进行霍夫线变换
	std::vector<cv::Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
	cv::HoughLinesP(black_img, lines, 100, PI/180, 60, 1500, 100 );
    if(lines.size()==0){
        return -400;
    }else{
        std::vector<double> angles(lines.size());
        for( size_t i = 0; i < lines.size(); i++ ){
            cv::Vec4i l = lines[i];
            angles.push_back(atan2(l[1]-l[3],l[0]-l[2])/PI*180);
        }
        std::sort(angles.begin(),angles.end());
        return -angles[lines.size()/2];	
    }
}
