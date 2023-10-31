#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <json/json.h>

using namespace cv;
using namespace std;

string object_models_path = "/home/jaka/catkin_ws/src/jakapbd/models/objects502.json";

vector<int> mask_paras(6);

// 输入图像
Mat img;
// 灰度值归一化
Mat bgr;
// HSV图像
Mat hsv;
// 色相
int hmin = 0;
int hmin_Max = 360;
int hmax = 360;
int hmax_Max = 360;
// 饱和度
int smin = 0;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
// 亮度
int vmin = 0;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 255;
// 显示原图的窗口
string windowName = "src";
// 输出图像的显示窗口
string dstName = "dst";
// 输出图像
Mat dst;

string objname, boxname;

// bool i = true;
// char a;

void imageconvertCB(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(img);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // char a = getchar();
    // if(a == 't'){
    //     i = false;
    // }

    /*
       namedWindow("img",0);
       resizeWindow("img",500,600);
       imshow("img", img);
       waitKey(0);
     */
    // i = false;
    // cout << "hello" << endl;
}

void callBack(int, void *)
{

    // 输出图像分配内存
    dst = Mat::zeros(img.size(), CV_32FC3);
    // 掩码
    Mat mask;
    inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
    // 只保留
    for (int r = 0; r < bgr.rows; r++)
    {
        for (int c = 0; c < bgr.cols; c++)
        {
            if (mask.at<uchar>(r, c) == 255)
            {
                dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
            }
        }
    }
    // 输出图像
    imshow(dstName, dst);
    // 保存图像
    dst.convertTo(dst, CV_8UC3, 255.0, 0);
    // vector<int> mask_paras;
    mask_paras.at(0) = (getTrackbarPos("hmin", dstName));
    mask_paras.at(1) = (getTrackbarPos("hmax", dstName));
    mask_paras.at(2) = (getTrackbarPos("smin", dstName));
    mask_paras.at(3) = (getTrackbarPos("smax", dstName));
    mask_paras.at(4) = (getTrackbarPos("vmin", dstName));
    mask_paras.at(5) = (getTrackbarPos("vmax", dstName));
    // waitKey(0);
    // cout<<"start saving the parameters..."<<endl;
    // ofstream output_file("hsv_thresh_parameters.txt");
    // ostream_iterator<int> output_iterator(output_file,"\n");
    // copy(mask_paras.begin(),mask_paras.end(),output_iterator);
    // imwrite("HSV_inRange.jpg", dst);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_paras502");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 10, imageconvertCB);

    ros::Rate loop_rate(30);

    ros::Duration(0.5).sleep();

    bool T;

    ros::param::set("/takepic", false);

    ros::param::get("/Object_type", objname);
    // ros::param::get("/Box_type",boxname);

    // cout<<"plz choose store object or box"<<endl;

    // int time_count = 0;
    system("gnome-terminal -x bash -c 'rosrun image_view image_view image:=/camera/color/image_raw __name:=imageviewer'");

    while (ros::ok())
    {
        // time_count++;
        ros::spinOnce();
        /*
           if(time_count > 300)
           {
           a = getchar();

           if(a == 't'){
        //i = false
        break;
        }
        }
         */
        /*
           char a = getchar();
           if(a == 't'){
           i = false;
           }*/
        // if(i==false)break;
        ros::param::get("takepic", T);
        if (T)
            break;

        loop_rate.sleep();
    }

    system("gnome-terminal -x rosnode kill imageviewer");
    if (ros::ok())
    {
        namedWindow("img", 0);
        resizeWindow("img", 500, 600);
        imshow("img", img);
        waitKey(0);

        // 彩色图像的灰度值归一化
        img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
        // 颜色空间转换
        blur(bgr, bgr, Size(3, 3));
        namedWindow("bgr0", 0);
        resizeWindow("bgr0", 600, 800);
        imshow("bgr0", bgr);
        waitKey(0);
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        // 定义输出图像的显示窗口
        namedWindow(dstName, WINDOW_GUI_EXPANDED);
        // 调节色相 H
        createTrackbar("hmin", dstName, &hmin, hmin_Max, callBack);
        createTrackbar("hmax", dstName, &hmax, hmax_Max, callBack);
        // 调节饱和度 S
        createTrackbar("smin", dstName, &smin, smin_Max, callBack);
        createTrackbar("smax", dstName, &smax, smax_Max, callBack);
        // 调节亮度 V
        createTrackbar("vmin", dstName, &vmin, vmin_Max, callBack);
        createTrackbar("vmax", dstName, &vmax, vmax_Max, callBack);
        callBack(0, 0);
        cout << "press s to save parameters" << endl;
        char key = waitKey(0);
        if (key == 's')
        {
            // ofstream output_file("hsv_thresh_parameters.txt");
            // ofstream output_file("/home/jaka/catkin_ws/src/jakapbd/src/red.txt");
            // ostream_iterator<int> output_iterator(output_file,"\n");
            // copy(mask_paras.begin(),mask_paras.end(),output_iterator);
            // 打开原有文件
            ifstream in(object_models_path, ios::binary);

            Json::Reader reader;
            Json::Value root;

            reader.parse(in, root);
            // 若该次演示物体已演示过，则覆盖
            // 若未演示过，则添加
            Json::Value item1, item2, item3, item4, item5, item6;
            item1["hmin"] = mask_paras.at(0);
            item2["hmax"] = mask_paras.at(1);
            item3["smin"] = mask_paras.at(2);
            item4["smax"] = mask_paras.at(3);
            item5["vmin"] = mask_paras.at(4);
            item6["vmax"] = mask_paras.at(5);
            // item7["box_name"]=boxname;

            if (root[objname].isNull())
            {
                root[objname].append(item1);
                root[objname].append(item2);
                root[objname].append(item3);
                root[objname].append(item4);
                root[objname].append(item5);
                root[objname].append(item6);
                // root[objname].append(item7);
            }
            else
            {
                root[objname][0] = item1;
                root[objname][1] = item2;
                root[objname][2] = item3;
                root[objname][3] = item4;
                root[objname][4] = item5;
                root[objname][5] = item6;
                // root[objname][6]=item7;
            }
            in.close();

            Json::StyledWriter sw;

            // 重新打开文件，并存储修改过的数据。
            ofstream os;
            os.open(object_models_path, ios::out | ios::ate);
            if (!os.is_open())
            {
                cout << "error:cannot find or create the file" << endl;
            }
            os << sw.write(root);
            os.close();
            cout << objname << "paras are saved in objects.json" << endl;
        }
        //    waitKey(0);
        for (int i = 0; i <= mask_paras.size() - 1; i++)
        {
            cout << mask_paras[i] << endl;
        }
    }

    return 0;
}
