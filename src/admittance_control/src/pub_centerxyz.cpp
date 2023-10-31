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
#include <admittance_control/centerpts2f.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/CameraInfo.h>

using namespace cv;
using namespace std;

Mat intrin_matrix;
float z_value = 0.0f;
float depth_max = 1500.0f;
float depth_min = 100.0f;
Point cp(320, 240);
int kernel_size = 3;
Mat img;
float base_T_op_array[16] = {-0.0225408, 0.999642, -0.014388, -0.477482, 0.997875, 0.0233764, 0.0608217, -0.0434973, 0.0611363, -0.0129864, -0.998045, 0.934635, 0, 0, 0, 1};
class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		// Topic1 you want to subscribe
		op_centerxyz_pub = n.advertise<geometry_msgs::Point32>("/op_centerxyz", 10);
		base_centerxyz_pub = n.advertise<geometry_msgs::Point32>("/base_centerxyz", 10);
		// depthimage_sub = n.subscribe("/camera/depth/image", 1, &SubscribeAndPublish::computeCB, this);
		depthimage_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &SubscribeAndPublish::computeCB, this);
		// Topic2 you want to subscribe
		centerxy_sub = n.subscribe("/centerxy", 1, &SubscribeAndPublish::centerCB, this);
		camerainfo_sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &SubscribeAndPublish::intrinsicCB, this);
	}

	void centerCB(const admittance_control::centerpts2f &center)
	{
		//    cout << "centerxy callback start" << endl;
		if (center.x < kernel_size || center.y < kernel_size)
		{
			cp.x = kernel_size;
			cp.y = kernel_size;
		}
		else
		{
			cp.x = (int)center.x;
			cp.y = (int)center.y;
		}
		//   cout << "centerxy callback end" << endl;
	}

	void computeCB(const sensor_msgs::ImageConstPtr &depth_msg)
	{
		// cout << "image_rect_raw callback start" << endl;
		cv_bridge::CvImagePtr depth_ptr;
		try
		{
			depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
			depth_ptr->image.copyTo(img);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		vector<Point> kernel;
		for (int i = 0; i <= kernel_size - 1; i++)
		{
			for (int j = 0; j <= kernel_size - 1; j++)
			{
				Point tmp;
				int k = (kernel_size - 1) / 2;
				tmp.x = cp.x - k + i;
				tmp.y = cp.y - k + j;
				kernel.push_back(tmp);
			}
		}

		cout << "kernel size: " << kernel.size() << endl;

		float z_value_total = 0.0f;
		// 遍历整个kernel读取深度求平均
		int nannumber = 0;

		for (int i = 0; i <= kernel.size() - 1; i++)
		{
			if (isnan(img.at<float>(kernel[i])) || img.at<float>(kernel[i]) > depth_max || img.at<float>(kernel[i]) < depth_min)
			{
				nannumber++;
			}
			else
			{
				// z_value_total = z_value_total + img.at<float>(kernel[i]);
				z_value_total = z_value_total + img.at<float>(kernel[i]) / 1000.0f;
			}
		}

		// cout<<nannumber<<endl;

		// float z_value;
		if (nannumber < pow(kernel_size, 2))
		{
			z_value = z_value_total / (pow(kernel_size, 2) - nannumber);
			// cout << "run" << endl;
		}
		cout << "z_value: " << z_value << endl;

		//  cout << "image_rect_raw callback end" << endl;
	}

	void intrinsicCB(const sensor_msgs::CameraInfo &info)
	{
		// cout << "camera_info callback start" << endl;
		float tmp[9] = {(float)info.K[0], (float)info.K[1], (float)info.K[2], (float)info.K[3], (float)info.K[4], (float)info.K[5], (float)info.K[6], (float)info.K[7], (float)info.K[8]};
		intrin_matrix = Mat(3, 3, CV_32FC1, tmp).clone();
		// cout<<intrin_matrix<<endl;
		Mat image_uvz = (Mat_<float>(3, 1) << (float)cp.x * z_value, (float)cp.y * z_value, z_value);
		Mat op_xyz_mat = intrin_matrix.inv() * image_uvz;
		// Mat op_xyz_mat_homo = (Mat_<float>(4,1)<<op_xyz_mat.at<float>(0,0),op_xyz_mat.at<float>(1,0),op_xyz_mat.at<float>(2,0)+24.75f,1.0f);

		// with offset
		// Mat op_xyz_mat_homo = (Mat_<float>(4,1)<<op_xyz_mat.at<float>(0,0),op_xyz_mat.at<float>(1,0),op_xyz_mat.at<float>(2,0)+0.04f,1.0f);

		// without offset
		Mat op_xyz_mat_homo = (Mat_<float>(4, 1) << op_xyz_mat.at<float>(0, 0), op_xyz_mat.at<float>(1, 0), op_xyz_mat.at<float>(2, 0), 1.0f);

		// cout<<"op_xyz_mat_homo: "<<op_xyz_mat_homo<<endl;
		Mat base_T_op = Mat(4, 4, CV_32FC1, base_T_op_array).clone();
		// cout<<"baseTop: "<<base_T_op<<endl;
		Mat base_xyz_mat_homo = base_T_op * op_xyz_mat_homo;
		// Mat base_xyz_mat = op_xyz_mat_homo;
		geometry_msgs::Point32 op_xyz;
		op_xyz.x = op_xyz_mat_homo.at<float>(0, 0);
		op_xyz.y = op_xyz_mat_homo.at<float>(1, 0);
		op_xyz.z = op_xyz_mat_homo.at<float>(2, 0);
		// cout<<"op_xyz.z: "<<op_xyz_mat_homo.at<float>(2,0)<<endl;
		op_centerxyz_pub.publish(op_xyz);
		geometry_msgs::Point32 base_xyz;
		base_xyz.x = base_xyz_mat_homo.at<float>(0, 0);
		base_xyz.y = base_xyz_mat_homo.at<float>(1, 0);
		base_xyz.z = base_xyz_mat_homo.at<float>(2, 0);
		base_centerxyz_pub.publish(base_xyz);
		// cout << "camera_info callback end" << endl;
	}

private:
	ros::NodeHandle n;
	ros::Publisher op_centerxyz_pub;
	ros::Publisher base_centerxyz_pub;
	ros::Subscriber depthimage_sub;
	ros::Subscriber centerxy_sub;
	ros::Subscriber camerainfo_sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compute_xyz502");

	ros::NodeHandle n1;

	// ros::Rate loop_rate(5);
	ros::Rate loop_rate(30);

	SubscribeAndPublish test;

	// ros::param::set("/centerxyz_start", false);

	bool centerxyz_start;

	// ros::Duration(3).sleep();

	while (ros::ok())
	{
		// ros::param::get("/centerxyz_start", centerxyz_start);
		// if(centerxyz_start == true){
		ros::spinOnce();
		//}
		loop_rate.sleep();
	}

	return 0;
}
