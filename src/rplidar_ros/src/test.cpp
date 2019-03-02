#define LOCAL
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <float.h>
#include<time.h>

using namespace cv;
using namespace std;
using namespace cv::ml;

#define WINDOW_NAME "【效果图窗口】"        //为窗口标题定义的宏
#define WINDOW_NAME1 "【蓝色图窗口】"        //为窗口标题定义的宏
#define ANGLE_NUM 360
#include <queue>
#define PI acos(-1)

int ksize = 5; //5         // ksize个点拟合一条直线
double deltaAngMax = 9; // 小于deltaAngMax认为可能是一条直线
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

Result calPosion(std::vector<std::vector<double> > &nodes, int count);
cv::Mat polyfit(std::vector<cv::Point2d> &in_point, int n);
void scan(double x,double y,double *everyCoord,int len);
double distance(cv::Point2d p1, cv::Point2d p2);
int deltaAngle(int a,int b);
void fit_line(std::vector<double> &kb,std::vector<cv::Point2d> &tempVec);

double rplidarSimmulate[ANGLE_NUM][2];
double everyCoord[ANGLE_NUM];
double xLimited = 3550;
double yLimited=700;
int main(){
//  一共有17289个数字
    #ifdef LOCAL
    freopen("/home/zhanggang/myC++/myC++/data32.txt", "r", stdin);
    freopen("/home/zhanggang/myC++/myC++/data33.txt", "w", stdout);
    #endif // LOCAL
    std::vector<std::vector<double>> data(800, std::vector<double>(2));
    int error_num=0;
    int coor_num=0;
    srand((unsigned)time(NULL));


//    for()


//    double x=3551,y=701;
    Mat black_img = Mat::zeros(Size(4200, 2600), CV_8UC1);//黑色图像


//    }

    // 距离墙的最小距离为50
    double x=100,y=500;
    scan(x,y,everyCoord,ANGLE_NUM);
//    for(int i=0;i<ANGLE_NUM;i++){
//        cout<<i<<" "<<everyCoord[i]<<endl;
//
//    }
    double dip;
    int dip_int;

//    用于调试
//    dip_int=90;
//    for(int k=0; k<ANGLE_NUM; k++)
//    {
//        rplidarSimmulate[k][0]=(double)k+0.1*(rand() / double(RAND_MAX)-0.5);
//        rplidarSimmulate[k][1]=everyCoord[(k+dip_int)%ANGLE_NUM]*(1+0.01*(rand() / double(RAND_MAX)-0.5)*2);
//        cout<<rplidarSimmulate[k][0]<<" " <<rplidarSimmulate[k][1]<<endl;
//    }


    //用于调试
//    clock_t startTime,endTime;
//	startTime = clock();
//    dip_int=42;
//    for(int k=0; k<ANGLE_NUM; k++)
//    {
//        rplidarSimmulate[k][0]=(double)k+0.1*(rand() / double(RAND_MAX)-0.5);
//        rplidarSimmulate[k][1]=everyCoord[(k+dip_int)%ANGLE_NUM]*(1+0.008*(rand() / double(RAND_MAX)-0.5));
//    }
//
//
//    int ccount=ANGLE_NUM;
//    int len = 0;
//    for (int i = 0; i < ccount; i++)
//    {
//
//        if (rplidarSimmulate[i][1] < 4600 && rplidarSimmulate[i][1]!=0.0)
//        {
//            data[len][0] = rplidarSimmulate[i][0];
//            data[len][1] = rplidarSimmulate[i][1];
//            ++len;
//        }
//    }
//    // ROS_INFO(": [allNode count:%d  ]", count);
//    len=len-1;
//    Result r;
//    r = calPosion(data, len);
//    int temp_dip;
//    if(dip_int>180)
//    {
//        temp_dip=dip_int-360;
//    }
//    else{
//        temp_dip=dip_int;
//    }
//    cout<<r.x<<" "<<r.y<<" "<<-r.ang<<endl;
//    if(r.x<0)
//    {
////                        cout<<x<<" "<<y<<" "<<dip_int<<endl;
//        return 0;
//    }
//    if(abs(r.x-x)<20 && abs(r.y-y)<20 && deltaAngle((int)(-r.ang+180),(int)(temp_dip+180))<15)
//    {
//
//    }
//    else{
////                        cout<<x<<" "<<y<<" "<<dip_int<<endl;
//        error_num++;
//    }
//    endTime = clock();
//	cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
//    cout<<error_num<<endl;
//    return 0;

    //用于调试3530 1170 270
//    x=3530;
//    y=1170;
//    scan(x,y,everyCoord,ANGLE_NUM);
//    dip_int=270;
//    for(int k=0; k<ANGLE_NUM; k++)
//    {
//        double r=sqrt((-2)*log(rand() / double(RAND_MAX)));
//        r=r*cos(2*PI*rand() / double(RAND_MAX));
//
//        rplidarSimmulate[k][0]=(double)k+0.04*r;// (rand() / double(RAND_MAX)-0.5)
////                        r=sqrt((-2)*log(rand() / double(RAND_MAX)));
////                        r=r*cos(2*PI*rand() / double(RAND_MAX));
//        rplidarSimmulate[k][1]=everyCoord[(k+dip_int)%ANGLE_NUM]*(1+0.008*(rand() / double(RAND_MAX)-0.5));
//        cout<<rplidarSimmulate[k][0]<<" "<<rplidarSimmulate[k][1]<<endl;
//    }
//    return 0;
//
//    int ccount=ANGLE_NUM;
//    int len = 0;
//    for (int i = 0; i < ccount; i++)
//    {
//
//        if (rplidarSimmulate[i][1] < 4600 && rplidarSimmulate[i][1]!=0.0)
//        {
//            data[len][0] = rplidarSimmulate[i][0];
//            data[len][1] = rplidarSimmulate[i][1];
//            ++len;
//        }
//    }

////    int ccount=ANGLE_NUM;
////    int len = 0;
////    for (int i = 0; i < 125; i++)//ccount
////    {
////        double a,b;
////        cin>>a>>b;
////        if (b < 4600 && b!=0.0)
////        {
////            data[len][0] = a;
////            data[len][1] = b;
////            ++len;
////        }
////    }
////
////    // ROS_INFO(": [allNode count:%d  ]", count);
////    len=len-1;
////    Result r;
////    r = calPosion(data, len);
////    int temp_dip;
////    if(dip_int>180)
////    {
////        temp_dip=dip_int-360;
////    }
////    else{
////        temp_dip=dip_int;
////    }
////    cout<<r.x<<" "<<r.y<<" "<<-r.ang<<endl;
////    if(r.x<0)
////    {
//////                        cout<<x<<" "<<y<<" "<<dip_int<<endl;
////        return 0;
////    }
////    if(abs(r.x-x)<20 && abs(r.y-y)<20 && deltaAngle((int)(-r.ang+180),(int)(temp_dip+180))<15)
////    {
////
////    }
////    else{
//////                        cout<<x<<" "<<y<<" "<<dip_int<<endl;
////        error_num++;
////    }
//////    endTime = clock();
//////	cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
////    cout<<error_num<<endl;
////    return 0;

//    int num=0;
//    for(double x=70; x<3550; x+=20)
//    {
//        for(double y=70; y<2600; y+=20)
//        {
//            if(sqrt(3)/3*x-y+1653>0 && -sqrt(3)/3*x-y+3464>0)
//            {
//                num++;
//            }
//        }
//    }
//    cout<<num<<endl;
//    return 0;



    double distance_max=0;

    for(double x=70;x<3550;x+=20){
        for(double y=70;y<2600;y+=20){
            if(sqrt(3)/3*x-y+1653>0 && -sqrt(3)/3*x-y+3464>0){
                coor_num++;
                scan(x,y,everyCoord,ANGLE_NUM);
                for(dip_int=0;dip_int<ANGLE_NUM;dip_int+=6){
//                    angleScan(rplidarSimmulate,)
                    for(int k=0;k<ANGLE_NUM;k++){
                        double r=sqrt((-2)*log(rand() / double(RAND_MAX)));
                        r=r*cos(2*PI*rand() / double(RAND_MAX));

                        rplidarSimmulate[k][0]=(double)k+0.04*r;// (rand() / double(RAND_MAX)-0.5)
//                        r=sqrt((-2)*log(rand() / double(RAND_MAX)));
//                        r=r*cos(2*PI*rand() / double(RAND_MAX));
                        rplidarSimmulate[k][1]=everyCoord[(k+dip_int)%ANGLE_NUM]*(1+0.008*(rand() / double(RAND_MAX)-0.5));
                    }


                    int ccount=ANGLE_NUM;
                    int len = 0;
                    for (int i = 0; i < ccount; i++)
                    {

                        if (rplidarSimmulate[i][1] < 4600 && rplidarSimmulate[i][1]!=0.0)
                        {
                            data[len][0] = rplidarSimmulate[i][0];
                            data[len][1] = rplidarSimmulate[i][1];
                            ++len;
                        }
                    }
                    // ROS_INFO(": [allNode count:%d  ]", count);
                    len=len-1;
                    Result r;
                    r = calPosion(data, len);
                    int temp_dip;
                    if(dip_int>180){
                        temp_dip=dip_int-360;
                    }else{
                        temp_dip=dip_int;
                    }
                    if(r.x<0){
                        cout<<0<<"  "<<x<<" "<<y<<" "<<dip_int<<endl;// <<" "<<r.x<<" "<<r.y<< " "<<-r.ang
                        cout<<len<<endl;
                        for(int i=0;i<len;i++){
                            cout<<data[i][0]<<" "<<data[i][1]<<endl;
                        }
                        error_num++;
                        continue;
                    }
                    if((abs(r.x-x)<20||abs(r.x-x)<0.05*x) && (abs(r.y-y)<20||abs(r.y-y)<0.05*x) && deltaAngle((int)(-r.ang+180),(int)(temp_dip+180))<15){
//                    cout<<1<<" "<<x<<" "<<y<<" "<<dip_int<<" "<<r.x<<" "<<r.y<< " "<<-r.ang<<endl;

                    }else{
                        cout<<1<<"  "<<x<<" "<<y<<" "<<dip_int<<" "<<r.x<<" "<<r.y<< " "<<-r.ang<<endl;
                        cout<<len<<endl;
                        for(int i=0;i<len;i++){
                            cout<<data[i][0]<<" "<<data[i][1]<<endl;
                        }
                        error_num++;
                    }

                }

//                black_img.at<uchar>(y,x)=255;
//                if(sqrt(x*x+y*y)>distance_max){
//                    distance_max=sqrt(x*x+y*y);
//                }
            }

        }
    }
//    cout<<"distance_max:"<<distance_max<<endl;
    cout<<"error_num:"<<error_num<<endl;

    cout<<"coor_num:"<<coor_num<<endl;
    cout<<"right:"<<1-(double)error_num/(coor_num*360/6)<<endl;


    Mat img = imread("//home//zhanggang//文档//testopencv//testopencv//1.png");
    Mat dst=Mat::zeros(Size(420, 260), CV_8UC1);
    cv::resize(black_img, dst, dst.size());
    imshow("lena", dst);
    waitKey(0);
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
            if(isnan(r.x)){
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
            if(isnan(r.x)){
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

void scan(double x,double y,double *everyCoord,int len){
    for(int i=0;i<len;i++){
        everyCoord[i]=0.0;
    }

    int theta=0;
    for(;theta<360;theta+=1){
        double angle=(double)theta*PI/180;
        double x0=x+y/tan(angle);
        double y0=y-x/tan(angle);
        if((x0>=0)){
            if(x0<=xLimited){
                everyCoord[theta]=abs(y/sin(angle));
            }
//            cout<<theta<<"->"<<x0<<"->"<<min(abs(y/sin(angle)),abs(x/cos(angle)))<<endl;
        }else{
//            cout<<theta<<"->"<<x0<<endl;
            break;
        }
    }
    for(;theta<360;theta+=1){
        double angle=(double)theta*PI/180;
        double x0=x+y/tan(angle);
        double y0=y-x/tan(angle-PI/2);
        if((y0<=yLimited)){
            if(y0>=0){
                everyCoord[theta]=abs(x/cos(angle));
            }
//            cout<<theta<<"->"<<y0<<"->"<<min(abs(y/sin(angle)),abs(x/cos(angle)))<<endl;
        }else{
//            cout<<theta<<"->"<<y0<<endl;
            break;
        }
    }
    for(int i=0;i<len;i++){
        if(everyCoord[i]<150){
            everyCoord[i]=0.0;
        }
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
