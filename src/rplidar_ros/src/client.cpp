/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
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
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "std_msgs/String.h"
#include <rplidar_ros/coordinate_msg.h>
//这些是后面添加的
#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <math.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <fstream>

#define PI acos(-1)

int ksize = 5;          // ksize个点拟合一条直线
double deltaAngMax = 5; // 小于deltaAngMax认为可能是一条直线
int deltaPointNum = 20;

struct Result
{
    double x;
    double y;
    double ang;
};

Result calPosion(std::vector<std::vector<double>> &nodes, int count);
cv::Mat polyfit(std::vector<cv::Point2d> &in_point, int n);
double distance(cv::Point2d p1, cv::Point2d p2);
// void pushData(std::vector<std::vector<double>> &file);

#define RAD2DEG(x) ((x)*180. / M_PI)

void scanCallback(const rplidar_ros::coordinate_msg::ConstPtr &scan)
{
    ROS_INFO(": [%05.1lf  %05.1lf  %03.1lf]", scan->coordinate_x, scan->coordinate_y, scan->angle);
    // std::vector<std::vector<double>> data(800, std::vector<double>(2));
    // int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    // clock_t t1;
    // t1 = clock();
    // int len = 0;
    // for (int i = 0; i < count; i++)
    // {
    //     // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    //     if (scan->ranges[i] * 1000 < 4600)
    //     {
    //         float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    //         data[len][0] = degree;
    //         data[len][1] = scan->ranges[i] * 1000;
    //         ++len;
    //     }
    // }
    // Result r;
    // r = calPosion(data, len - 1);
    // ROS_INFO(": [%05.1lf  %05.1lf  %03.1lf  %lf]", r.x, r.y, -r.ang, (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<rplidar_ros::coordinate_msg>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}

Result calPosion(std::vector<std::vector<double>> &nodes, int count)
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
        for (int i = 0; i < XYdata.size() - ksize; i += ksize)
        {
            copy(XYdata.begin() + i, XYdata.begin() + i + ksize, temp.begin());
            cv::Mat Mat_k = polyfit(temp, 1);
            double b = Mat_k.at<double>(0, 0);
            double k = Mat_k.at<double>(1, 0);
            pList[i / ksize] = std::atan(k) * 180 / PI;
        }
        // ROS_INFO("[pList1.size():%d]", pList.size());

        //todo 输出调试
        /*cout << "pList:" << endl;
		for (int i = 0; i < pList.size(); i++)
		{
			cout <<i<<" "<< pList[i] << endl;
		}*/

        std::vector<std::vector<int>> region;
        // TODO 下面一段是有问题的
        for (int i = 0; i < pList.size();)
        {
            std::vector<int> col;
            int j = i;
            col.push_back(j);
            j = j + 1;
            for (; j < pList.size() - 1;)
            {
                if (std::abs(pList[(j + 1)] - pList[j]) < deltaAngMax || std::abs(std::abs(pList[(j + 1)] - pList[j]) - 180) < deltaAngMax)
                {
                    col.push_back(j);
                    j++;
                }
                else
                {
                    j++;
                    break;
                }
            }
            region.push_back(col);
            i = j;
        }

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

        /*cout << "region:" << endl;
		for (int i = 0; i < region.size(); i++)
		{
			for (int j = 0; j < region[i].size(); j++)
			{
				cout << region[i][j] << " ";
			}
			cout << endl;
		}*/

        //int pListLen = pList.size()-1;
        int tempInt = pList.size() - 1;
        if (std::abs(pList[(tempInt + 1) % pList.size()] - pList[tempInt]) < deltaAngMax || std::abs(std::abs(pList[(tempInt + 1) % pList.size()] - pList[tempInt]) - 180) < deltaAngMax)
        {
            region[region.size() - 1].insert(region[region.size() - 1].end(), region[0].begin(), region[0].end());
            region.erase(region.begin());
        }
        //删去过短的部分
        for (int i = region.size() - 1; i >= 0; i--)
        {
            if (region[i].size() < 3)
            {
                region.erase(region.begin() + i);
            }
        }

        // ROS_INFO("[region2.size():%d]", region.size());
        /*cout << "region:" << endl;
		for (int i = 0; i < region.size(); i++)
		{
			for (int j = 0; j < region[i].size(); j++)
			{
				cout << region[i][j] << " ";
			}
			cout << endl;
		}*/

        std::vector<std::vector<int>> lines;
        for (int i = 0; i < region.size(); i++)
        {
            std::vector<int> aline;
            for (int j = 1; j < region[i].size() - 1; j++)
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
        /*cout << "lines:" << endl;
		for (int i = 0; i < lines.size(); i++)
		{
			for (int j = 0; j < lines[i].size(); j++)
			{
				cout << lines[i][j] << " ";
			}
			cout << endl;
		}*/

        std::vector<std::vector<double>> allLine;
        for (int i = 0; i < lines.size(); i++)
        {
            std::vector<cv::Point2d> tempVec;
            if (lines[i].size() == lines[i][lines[i].size() - 1] - lines[i][0] + 1)
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

            cv::Mat Mat_k = polyfit(tempVec, 1);
            double b = Mat_k.at<double>(0, 0);
            double k = Mat_k.at<double>(1, 0);
            //pList[i / ksize] = k;
            std::vector<double> kb(2);
            kb[0] = atan(k) * 180 / PI;
            kb[1] = std::abs(b) / sqrt(k * k + 1);
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


        /*cout << "allLine:" << endl;
		for (int i = 0; i < allLine.size(); i++)
		{
			for (int j = 0; j < allLine[i].size(); j++)
			{
				cout << allLine[i][j] << " ";
			}
			cout << endl;
		}*/

        // 提高精度到时候可以用加权最小二乘法，因为越近精度越高，随机
        // 一个角度减去另一个角度等于0度或者180度,而且距离原点距离相等，则判定属于直线
        std::vector<std::vector<int>> allDeltaAng;
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
                    cv::Mat Mat_k = polyfit(tempVec, 1);
                    double b1 = Mat_k.at<double>(0, 0);
                    double k = Mat_k.at<double>(1, 0);
                    //pList[i / ksize] = k;
                    std::vector<double> kb(2);
                    kb[0] = std::atan(k) * 180 / PI;
                    kb[1] = std::abs(b1) / std::sqrt(k * k + 1);
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
        /*cout << "allLine:" << endl;
		for (int i = 0; i < allLine.size(); i++)
		{
			for (int j = 0; j < allLine[i].size(); j++)
			{
				cout << allLine[i][j] << " ";
			}
			cout << endl;
		}
		cout << "lines:" << endl;
		for (int i = 0; i < lines.size(); i++)
		{
			for (int j = 0; j < lines[i].size(); j++)
			{
				cout << lines[i][j] << " ";
			}
			cout << endl;
		}*/

        std::vector<int> suitLine;
        for (int i = 0; i < allLine.size(); i++)
        {
            if (std::abs(std::abs(allLine[(i + 1) % allLine.size()][0] - allLine[i][0]) - 90) < deltaAngMax)
            { // 20
                suitLine.push_back(i);
            }
        }
        int ans;
        if (suitLine.size() == 0)
        {
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
                double distance = sqrt(std::pow(XYdata[a1].x - XYdata[a2].x, 2) + std::pow(XYdata[a1].y - XYdata[a2].y, 2)) + sqrt(std::pow(XYdata[b1].x - XYdata[b2].x, 2) + std::pow(XYdata[b1].y - XYdata[b2].y, 2));
                lineLength.push_back(distance);
            }
            std::vector<double>::iterator biggest = std::max_element(std::begin(lineLength), std::end(lineLength));
            ans = suitLine[std::distance(std::begin(lineLength), biggest)];
        }
        if (distance(XYdata[lines[(ans + 1) % lines.size()][lines[(ans + 1) % lines.size()].size() - 1]], XYdata[lines[ans][0]]) > distance(XYdata[lines[(ans + 1) % lines.size()][0]], XYdata[lines[ans][lines[ans].size() - 1]]))
        {
            struct Result r;
            r.x = allLine[(ans + 1) % allLine.size()][1];
            r.y = allLine[ans][1];

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
            r.x = allLine[ans][1];

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