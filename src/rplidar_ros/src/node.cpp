/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT count++;SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
//1ms

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
int deltaAngle(int a,int b);
void fit_line(std::vector<double> &kb,std::vector<cv::Point2d> &tempVec);

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace rp::standalone::rplidar;

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
                ROS_INFO(": [node:%05.1lf  %05.1lf  %03.1lf]", r.x, r.y, -r.ang);
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
                    double angle_increment=45-r.ang;
                    
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
                    if(dataSelect(selectData,r.x) ){//&& angleSelect(selectAngle,-r.ang)
                        msg.coordinate_x = r.x;
                        msg.coordinate_y = r.y;
                        msg.angle = -r.ang;

                        scan_pub.publish(msg);
                    }

                    
                    selectData.pop();
                    // selectAngle.pop();                    
                    selectData.push(r.x);
                    // selectAngle.push(-r.ang);
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
        // ROS_INFO("[count:%d]", count);
        std::vector<cv::Point2d> XYdata(count);
        //std::vector<std::vector <double> > data(800, std::vector<double>(2));
        for (int pos = 0; pos < count; ++pos)
        {
            XYdata[pos].x = nodes[pos][1] * cos(nodes[pos][0] * PI / 180);
            XYdata[pos].y = nodes[pos][1] * sin(nodes[pos][0] * PI / 180);
            /*XYdata[pos][0] = nodes[pos][1] * cos(nodes[pos][0]);
			XYdata[pos][1] = nodes[pos][1] * sin(nodes[pos][0]);*/
        }
        //cv::Mat Mat_k = polyfit(XYdata, 1);

        std::vector<cv::Point2d> temp(ksize);
        std::vector<double> pList(XYdata.size() / ksize);
        int c=0;
        if(XYdata.size()%ksize==0){
            c=XYdata.size();
        }else{
            c=XYdata.size() - ksize;
        }
        // TODO 对于余下的点被浪费的问题，当点数大于3时，可以将前面的点复制一下，添加到现有的里面
        for (int i = 0; i < c; i += ksize)//for (int i = 0; i < XYdata.size() - ksize; i += ksize)
        {
            double dang=nodes[i+ksize-1][0]-nodes[i][0];
            if(dang<0) dang+=360;
            if(dang>45){
                pList[i / ksize] = 4000;
                continue;
            }
            copy(XYdata.begin() + i, XYdata.begin() + i + ksize, temp.begin());
            cv::Mat Mat_k = polyfit(temp, 1);
            double b = Mat_k.at<double>(0, 0);
            double k = Mat_k.at<double>(1, 0);
            pList[i / ksize] = std::atan(k) * 180 / PI;
        }
        // ROS_INFO("[pList1.size():%d]", pList.size());

        //todo 输出调试
//        cout<<count<<endl;
//        cout << "pList:" << endl;
//		for (int i = 0; i < pList.size(); i++)
//		{
//			cout <<i<<" "<< pList[i] << endl;
//		}

        std::vector<std::vector<int> > region;
        // TODO 下面一段是有问题的
//        for (int i = 0; i < pList.size();)
//        {
//            std::vector<int> col;
//            int j = i;
//            col.push_back(j);
//            j = j + 1;
//            for (; j < pList.size() - 1;)
//            {
//                if (std::abs(pList[(j + 1)] - pList[j]) < deltaAngMax || std::abs(std::abs(pList[(j + 1)] - pList[j]) - 180) < deltaAngMax)
//                {
//                    col.push_back(j);
//                    j++;
//                }
//                else
//                {
//                    j++;
//                    break;
//                }
//            }
//            region.push_back(col);
//            i = j;
//        }

        for (int i = 0; i < pList.size();)
        {
            std::vector<int> col;
            col.push_back(i);
            int j = i + 1;
            for (; j < pList.size();)
            {
                if (std::abs(pList[(j)] - pList[j - 1]) < deltaAngMax || std::abs(std::abs(pList[(j)] - pList[j - 1]) - 180) < deltaAngMax)
                {
                    col.push_back(j);
                    j++;
                }
                else
                {
                    break;
                }
            }

            // for(int j=i,j<pList.size()-1;){
            //     if (std::abs(pList[(j + 1)] - pList[j]) < deltaAngMax
            // 		|| std::abs(std::abs(pList[(j + 1) ] - pList[j]) - 180) < deltaAngMax) {
            //         col.push_back(j+1);
            //         j++;
            //     }else{
            //         j++;
            //         break;
            //     }
            // }
            region.push_back(col);
            i = j;
        }
        // ROS_INFO("[region1.size():%d]", region.size());

//        cout << "region:" << endl;
//		for (int i = 0; i < region.size(); i++)
//		{
//			for (int j = 0; j < region[i].size(); j++)
//			{
//				cout << region[i][j] << " ";
//			}
//			cout << endl;
//		}

        //int pListLen = pList.size()-1;
        int tempInt = pList.size() - 1;
        if (std::abs(pList[(tempInt + 1) % pList.size()] - pList[tempInt]) < deltaAngMax || std::abs(std::abs(pList[(tempInt + 1) % pList.size()] - pList[tempInt]) - 180) < deltaAngMax)
        {
            region[region.size() - 1].insert(region[region.size() - 1].end(), region[0].begin(), region[0].end());
            region.erase(region.begin());
        }
        int l=pList.size();
        //删去过短的部分
        for (int i = region.size() - 1; i >= 0; i--)
        {
            if (region[i].size() < 2)//< 2
            {
                int k=region[i][0];
                if(pList[k]==4000){
                    region.erase(region.begin() + i);
//                    i--;
                    continue;
                }else if(std::abs(std::abs(pList[(k-1+l)%l]-pList[(k+1)%l])-90)<deltaAngMax){
                    region.erase(region.begin() + i);
//                    i-=2;
                    i-=1;
                    continue;
                }
            }
        }

        // ROS_INFO("[region2.size():%d]", region.size());
//        cout << "region:" << endl;
//		for (int i = 0; i < region.size(); i++)
//		{
//			for (int j = 0; j < region[i].size(); j++)
//			{
//				cout << region[i][j] << " ";
//			}
//			cout << endl;
//		}

        std::vector<std::vector<int> > lines;
        for (int i = 0; i < region.size(); i++)
        {
            std::vector<int> aline;
            for (int j = 0; j < region[i].size(); j++)//for (int j = 1; j < region[i].size() - 1; j++)
            {
                int t = region[i][j];
                for (int k = 0; k < 5; k++)
                {
                    aline.push_back(t * 5 + k);
                }
            }
            lines.push_back(aline);
        }
        // ROS_INFO("[lines.size():%d]", lines.size());
//        cout << "lines:" << endl;
//		for (int i = 0; i < lines.size(); i++)
//		{
//			for (int j = 0; j < lines[i].size(); j++)
//			{
//				cout << lines[i][j] << " ";
//			}
//			cout << endl;
//		}

        std::vector<std::vector<double> > allLine;
        for (int i = 0; i < lines.size(); i++)
        {
            std::vector<cv::Point2d> tempVec;
            int last=lines[i].size() - 1;
            if (lines[i].size() == lines[i][last] - lines[i][0] + 1)
            {
                copy(XYdata.begin() + lines[i][0], XYdata.begin() + lines[i][0] + lines[i].size(), std::back_inserter(tempVec)); // + lines[i].size()
            }
            else
            {
                for (int j = 0; j < lines[i].size(); j++)
                {
                    tempVec.push_back(XYdata[lines[i][j]]);
                }
            }
            std::vector<double> kb(4);

            fit_line(kb,tempVec);
//
//            cv::Mat Mat_k = polyfit(tempVec, 1);
//            double b = Mat_k.at<double>(0, 0);
//            double k = Mat_k.at<double>(1, 0);
//            //pList[i / ksize] = k;
//
//            kb[0] = atan(k) * 180 / PI;
//            kb[1] = std::abs(b) / sqrt(k * k + 1);
            allLine.push_back(kb);
        }
        // ROS_INFO("[allLine.size():%d]", allLine.size());
        if (allLine.size() < 2)
        {
            struct Result r;
            r.y = -1001;
            r.x = -1001;
            r.ang = 400;
            return r;
        }
        // ROS_INFO("[allLine is]");
        // for (int i = 0; i < allLine.size(); i++)
        // {
        //     for (int j = 0; j < allLine[i].size(); j++)
        //     {
        //         ROS_INFO("[allLine %d %d:%lf]", i, j, allLine[i][j]);
        //     }
        // }

//        cout << "allLine:" << endl;
//		for (int i = 0; i < allLine.size(); i++)
//		{
//			for (int j = 0; j < allLine[i].size(); j++)
//			{
//				cout << allLine[i][j] << " ";
//			}
//			cout << endl;
//		}

        // 提高精度到时候可以用加权最小二乘法，因为越近精度越高，随机
        // 一个角度减去另一个角度等于0度或者180度,而且距离原点距离相等，则判定属于直线
        std::vector<std::vector<int> > allDeltaAng;
        for (int i = allLine.size() - 2; i >= -1; i--)
        {

            int a = (i + allLine.size()) % allLine.size();
            int b = (i + 1 + allLine.size()) % allLine.size();
            // ROS_INFO("a,b:%d,%d", a, b);
            std::vector<cv::Point2d> tempVec;
            if (std::abs(lines[b][0] - lines[a][lines[a].size() - 1]) < deltaPointNum &&
                std::abs(allLine[b][1] - allLine[a][1]) / allLine[a][1] < 0.05)
            {
                if (std::abs(allLine[b][0] - allLine[a][0]) < deltaAngMax ||
                    std::abs(std::abs(allLine[b][0] - allLine[a][0]) - 180) < deltaAngMax)
                {
                    if (lines[a].size() == lines[a][lines[a].size() - 1] - lines[a][0] + 1)
                    {
                        std::copy(XYdata.begin() + lines[a][0], XYdata.begin() + lines[a][0] + lines[a].size(), std::back_inserter(tempVec));
                    }
                    else
                    {
                        for (int j = 0; j < lines[a].size(); j++)
                        {
                            tempVec.push_back(XYdata[lines[a][j]]);
                        }
                    }
                    // ROS_INFO("2a,b:%d,%d", a, b);
                    /*for (int j = 0; j < lines[a].size(); j++) {
						tempVec.push_back(XYdata[lines[a][j]]);
					}*/
                    if (lines[b].size() == lines[b][lines[b].size() - 1] - lines[b][0] + 1)
                    {
                        std::copy(XYdata.begin() + lines[b][0], XYdata.begin() + lines[b][0] + lines[b].size(), std::back_inserter(tempVec));
                    }
                    else
                    {
                        for (int j = 0; j < lines[b].size(); j++)
                        {
                            tempVec.push_back(XYdata[lines[b][j]]);
                        }
                    }
                    // ROS_INFO("3a,b:%d,%d", a, b);
                    /*for (int j = 0; j < lines[b].size(); j++) {
						tempVec.push_back(XYdata[lines[b][j]]);
					}*/
                    std::copy(lines[b].begin(), lines[b].end(), std::back_inserter(lines[a]));
                    lines.erase(lines.begin() + b);
                    std::vector<double> kb(4);
                    fit_line(kb,tempVec);
//                    cv::Mat Mat_k = polyfit(tempVec, 1);
//                    double b1 = Mat_k.at<double>(0, 0);
//                    double k = Mat_k.at<double>(1, 0);
//                    //pList[i / ksize] = k;
//                    std::vector<double> kb(2);
//                    kb[0] = std::atan(k) * 180 / PI;
//                    kb[1] = std::abs(b1) / std::sqrt(k * k + 1);
                    allLine.erase(allLine.begin() + b);
                    /*allLine.erase(allLine.begin() + a);
					allLine.insert(allLine.begin() + a, kb);*/
                    allLine[a] = kb;
                }
            }

            if (allLine.size() < 2)
            {



                struct Result r;
                r.y = -1001;
                r.x = -1001;
                r.ang = 400;
                return r;
            }
        }
        // ROS_INFO("[allLine2.size():%d]", allLine.size());
        // ROS_INFO("[lines2.size():%d]", lines.size());
//        cout << "allLine:" << endl;
//		for (int i = 0; i < allLine.size(); i++)
//		{
//			for (int j = 0; j < allLine[i].size(); j++)
//			{
//				cout << allLine[i][j] << " ";
//			}
//			cout << endl;
//		}
//		cout << "lines:" << endl;
//		for (int i = 0; i < lines.size(); i++)
//		{
//			for (int j = 0; j < lines[i].size(); j++)
//			{
//				cout << lines[i][j] << " ";
//			}
//			cout << endl;
//		}

        std::vector<int> suitLine;
        for (int i = 0; i < allLine.size(); i++)
        {
            if (std::abs(std::abs(allLine[(i + 1) % allLine.size()][0] - allLine[i][0]) - 90) < 15)//deltaAngMax
            { // 20
                suitLine.push_back(i);
            }
        }

//        cout<<"suitLine:"<<endl;
//        for(int i=0;i<suitLine.size();i++){
//            cout<<suitLine[i]<<endl;
//        }

        int ans;
        if (suitLine.size() == 0)
        {

            // for (int i = 0; i < allLine.size(); i++)
            // {
            //     if (std::abs(std::abs(allLine[(i + 2) % allLine.size()][0] - allLine[i][0]) - 90) < deltaAngMax)
            //     { // 20
            //         // suitLine.push_back(i);
            //         ROS_INFO("[real have answer]");
            //     }
            // }
            struct Result r;
            r.y = -1001;
            r.x = -1001;
            r.ang = 400;
            return r;
        }

        if (suitLine.size() == 1 || (allLine.size() == 2 && suitLine.size() == 2))
        {
            ans = suitLine[0];
        }
        else
        {
            std::vector<double> lineLength;
            for (int i = 0; i < suitLine.size(); i++)
            {
                int tempInt = suitLine[i];
                int a1 = lines[tempInt][0];
                int a2 = lines[tempInt][lines[tempInt].size() - 1];
                int b1 = lines[(tempInt + 1) % allLine.size()][0];
                int b2 = lines[(tempInt + 1) % allLine.size()][lines[(tempInt + 1) % allLine.size()].size() - 1];
                double distance = sqrt(std::pow(XYdata[a1].x - XYdata[a2].x, 2) + std::pow(XYdata[a1].y - XYdata[a2].y, 2)) +
                sqrt(std::pow(XYdata[b1].x - XYdata[b2].x, 2) + std::pow(XYdata[b1].y - XYdata[b2].y, 2));
                lineLength.push_back(distance);
//                cout<<i<<" "<<a1<<" "<<a2<<" "<<b1<<" "<<b2<<" "<<distance<<endl;
            }

            std::vector<double>::iterator biggest = std::max_element(std::begin(lineLength), std::end(lineLength));
            ans = suitLine[std::distance(std::begin(lineLength), biggest)];
        }
//        cout<<ans<<endl;

        if (distance(XYdata[lines[(ans + 1) % lines.size()][lines[(ans + 1) % lines.size()].size() - 1]],
        XYdata[lines[ans][0]]) > distance(XYdata[lines[(ans + 1) % lines.size()][0]],
        XYdata[lines[ans][lines[ans].size() - 1]]))
        {
            struct Result r;

            r.y = allLine[ans][1];
//            r.x = allLine[(ans + 1) % allLine.size()][1];

            double k1=allLine[ans][2];
            double b1=allLine[ans][3];
            double k2=allLine[(ans + 1) % allLine.size()][2];
            double b2=allLine[(ans + 1) % allLine.size()][3];
            if(std::abs(k1)>30 || std::abs(k2)>30){
                r.x = allLine[(ans + 1) % allLine.size()][1];
            }else{
                double x_cood=(b2-b1)/(k1-k2);
                double y_cood;
                if(std::abs(k1)>std::abs(k2)){
                    y_cood=k2*x_cood+b2;
                }else{
                    y_cood=k1*x_cood+b1;
                }
                r.x=sqrt(x_cood*x_cood+y_cood*y_cood-r.y*r.y);
            }
            if(std::isnan(r.x)){
                r.x = allLine[(ans + 1) % allLine.size()][1];
            }


            int b = lines[ans][0];
            int a = lines[ans][lines[ans].size() - 1];

            double theta = std::atan2(XYdata[b].y - XYdata[a].y, XYdata[b].x - XYdata[a].x) * 180 / PI;
            if (std::abs(std::abs(theta - allLine[ans][0]) - 180) < 45)
            {
                if (allLine[ans][0] < 0)
                    r.ang = allLine[ans][0] + 180;
                else
                {
                    r.ang = allLine[ans][0] - 180;
                }
            }
            else
            {
                r.ang = allLine[ans][0];
            }

            return r;
        }
        else
        {
            struct Result r;
            r.y = allLine[(ans + 1) % allLine.size()][1];
//            r.x = allLine[ans][1];


            double k1=allLine[ans][2];
            double b1=allLine[ans][3];
            double k2=allLine[(ans + 1) % allLine.size()][2];
            double b2=allLine[(ans + 1) % allLine.size()][3];
            if(std::abs(k1)>30 || std::abs(k2)>30){
                r.x = allLine[ans][1];
            }else{
                double x_cood=(b2-b1)/(k1-k2);
                double y_cood;
                if(std::abs(k1)>std::abs(k2)){
                    y_cood=k2*x_cood+b2;
                }else{
                    y_cood=k1*x_cood+b1;
                }
                r.x=sqrt(x_cood*x_cood+y_cood*y_cood-r.y*r.y);
            }
            if(std::isnan(r.x)){
                r.x = allLine[ans][1];
            }

            int b = lines[(ans + 1) % allLine.size()][0];
            int a = lines[(ans + 1) % allLine.size()][lines[(ans + 1) % allLine.size()].size() - 1];

            double theta = std::atan2(XYdata[b].y - XYdata[a].y, XYdata[b].x - XYdata[a].x) * 180 / PI;
            //cout << theta << " " << allLine[(ans + 1) % allLine.size()][0] << endl;
            if (std::abs(std::abs(theta - allLine[(ans + 1) % allLine.size()][0]) - 180) < 45)
            {
                if (allLine[(ans + 1) % allLine.size()][0] < 0)
                    r.ang = allLine[(ans + 1) % allLine.size()][0] + 180;
                else
                {
                    r.ang = allLine[(ans + 1) % allLine.size()][0] - 180;
                }
            }
            else
            {
                r.ang = allLine[(ans + 1) % allLine.size()][0];
            }
            /*if (allLine[(ans + 1) % allLine.size()][0] - allLine[ans][0] > 0) {
				r.ang = 180 - allLine[ans][0];

			}
			else
			{
				r.ang = allLine[(ans + 1) % allLine.size()][0];
			}*/
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