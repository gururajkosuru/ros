#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include <cv.h>
#include <highgui.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

using namespace std;
using namespace cv;
Mat displayData;
int width, height;
float resolution;


void convertEtoQ(Mat& Q, double pi, double theta, double si)
{
	Q.at<double>(0,0) = cos(pi/2)*cos(theta/2)*cos(si/2) + sin(pi/2)*sin(theta/2)*sin(si/2);
	Q.at<double>(1,0) = sin(pi/2)*cos(theta/2)*cos(si/2) - cos(pi/2)*sin(theta/2)*sin(si/2);
	Q.at<double>(2,0) = cos(pi/2)*sin(theta/2)*cos(si/2) + sin(pi/2)*cos(theta/2)*sin(si/2);
	Q.at<double>(3,0) = cos(pi/2)*cos(theta/2)*sin(si/2) - sin(pi/2)*sin(theta/2)*cos(si/2);

}

void setLocalToGlobal(geometry_msgs::Point P, double th)
{
        geometry_msgs::Pose gl;
        gl.position.x = P.x; gl.position.y = P.y; gl.position.z = 0.0;
//      cout<<"Level 1 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
        //Level 1-2
        gl.position.y = height - gl.position.y;
//      cout<<"Level 2 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
        //Level 2-3
        gl.position.x = gl.position.x - width/2;
        gl.position.y = gl.position.y - height/2;
//      cout<<"Level 3 at "<<gl.position.x<<" , "<<gl.position.y<<endl;
        //Level 3-4
        gl.position.x*=resolution;
        gl.position.y*=resolution;
//      cout<<"Level 4 at "<<gl.position.x<<" , "<<gl.position.y<<endl;

        Mat Q(4, 1, CV_64F);
        double pi=0, theta=0;
        double si = th*M_PI/180;

        convertEtoQ(Q, pi, theta, si);
	geometry_msgs::Pose global_location;
        global_location.position.x = gl.position.x;
        global_location.position.y = gl.position.y;
        global_location.position.z = gl.position.z;

        global_location.orientation.w = Q.at<double>(0, 0);
        global_location.orientation.x = Q.at<double>(1, 0);
        global_location.orientation.y = Q.at<double>(2, 0);
        global_location.orientation.z = Q.at<double>(3, 0);
	cout << global_location << endl;
}



void mouseEvent(int evt, int x, int y, int flags, void* param){
	if(evt==CV_EVENT_LBUTTONDOWN){
		printf("%d %d\n",x,y);
		int temp = x;
		x = y;
		y = temp;
		displayData.at<uchar>(x-1,y-1) = 155;
		displayData.at<uchar>(x-1,y) = 155;
		displayData.at<uchar>(x-1,y+1) = 155;
		displayData.at<uchar>(x,y-1) = 155;
		displayData.at<uchar>(x,y) = 155;
		displayData.at<uchar>(x,y+1) = 155;
		displayData.at<uchar>(x+1,y-1) = 155;
		displayData.at<uchar>(x+1,y) = 155;
		displayData.at<uchar>(x+1,y+1) = 155;
		imshow( "Display window", displayData );                   // Show our image inside it.
		geometry_msgs::Point P;
		P.x = 2*x;
		P.y = 2*y;
		setLocalToGlobal(P, 0.0);

	}
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::ServiceClient map_service_client_ = n.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv_map;

    if (map_service_client_.call(srv_map))
    {
	    ROS_INFO("Map service called successfully");
	    const nav_msgs::OccupancyGrid& map (srv_map.response.map);
	    cout << "rows: "<< map.info.width << " and cols: " <<  map.info.height << endl;
	    cout << "resolution: "<< map.info.resolution << endl;
	    cout << " origin " << map.info.origin << endl;
	    width = map.info.width;
	    height = map.info.height;
	    resolution = map.info.resolution;
	    geometry_msgs::Pose origin = map.info.origin;
	    
	    //do something with the map
	    CvSize size;
	    size.height = height ;
	    size.width = width;
	    Mat data(width, height, CV_8UC1);
	    displayData.create(width/2, height/2, CV_8UC1);
	    
	    for( int i = 0; i < width; i++)
		    for( int j = 0; j < height; j++)
			    data.at<uchar>(i, j) = map.data[i*width + j];
	    if(! data.data )                              // Check for invalid input
	    {
		    cout <<  "Could not open or find the image" << std::endl ;
		    return -1;
	    }
	    resize(data, displayData, Size(), 0.5, 0.5, CV_INTER_AREA);	
	    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
	    imshow( "Display window", displayData );                   // Show our image inside it.
	      //assigning the callback function for mouse events
	    cvSetMouseCallback("Display window", mouseEvent, 0);
	    waitKey(0);                                          // Wait for a keystroke in the window
	    return 0;

    }
    else
    {
	    ROS_ERROR("Failed to call map service");
	    return -1;
    }
    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin();

  return 0;
}
