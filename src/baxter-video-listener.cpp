#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpMeEllipse.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <baxter_core_msgs/EndpointState.h>

//  used for simulation tracking
#include <visp/vpSimulatorCamera.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpProjectionDisplay.h>
#include <visp/vpServoDisplay.h>


namespace enc = sensor_msgs::image_encodings;
baxter_core_msgs::EndpointState current_state;

static const char WINDOW[] = "Baxter block tracker";

int global_min_threshold=50;
int global_squareness_ratio=20;

void update_global_min_threshold(int,void*) 
{
	//do nothing
}

void update_global_squareness_ratio(int,void*) 
{
	//do nothing
}

class ImageConverter
{
	bool tracking;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
    ros::Subscriber endpt_sub_;
	vpHomogeneousMatrix cMo;

	vpMbEdgeTracker tracker;
	vpDisplayOpenCV *d;
	vpCameraParameters cam;
	char* ros_image_stream;
	bool firstFrame;

    public:
	ImageConverter(char* ros_img_stream, char* ros_endpoint_topic)
		: it_(nh_)
	{
		ros_image_stream = ros_img_stream;
		tracking = false;
		firstFrame = true;

		image_sub_ = it_.subscribe(ros_image_stream, 1, &ImageConverter::analyze_frame, this);
        endpt_sub_ = nh_.subscribe(ros_endpoint_topic, 1, &ImageConverter::endpt_grabber, this);

		cv::namedWindow(WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(WINDOW);
	}

    void endpt_grabber(const baxter_core_msgs::EndpointState& msg)
    {
        current_state = msg;
    }

	void analyze_frame(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//  Initialize the edge tracker if this is the first frame recieved
		if(firstFrame)
		{
			firstFrame = false;            
			init_the_edge_tracker(cv_ptr->image);
		}

		// now look for the cube in the most recent frame
		trackCube(cv_ptr->image);
	}

	void init_the_edge_tracker(cv::Mat img)
	{
		vpImage<unsigned char> I;

		vpImageConvert::convert(img, I);
		d = new vpDisplayOpenCV(I);

		vpDisplay::display(I);
		vpDisplay::flush(I);

		vpMe me;
		me.setMaskSize(5);
		me.setMaskNumber(180);
		me.setRange(8);
		me.setThreshold(10000);
		me.setMu1(0.5);
		me.setMu2(0.5);
		me.setSampleStep(4);
		me.setNbTotalSample(250);
		tracker.setMovingEdge(me);
		cam.initPersProjWithoutDistortion(839, 839, 325, 243);
		tracker.setCameraParameters(cam);
		tracker.setNearClippingDistance(0.1);
		tracker.setFarClippingDistance(100.0);
		tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
		tracker.setDisplayFeatures(true);
		tracker.setOgreVisibilityTest(false);   //  Set to true to break everything, but it works a lot better
		tracker.setAngleAppear(70);
		tracker.setAngleDisappear(80);
		tracker.loadModel("cube.cao");
		tracker.initClick(I, "cube.init");
	}

	private:
	void trackCube(cv::Mat frame)
	{
	  vpHomogeneousMatrix cdMo(0, 0, .75, 0, 0, 0);
	  tracker.getPose(cMo);
	  vpImage<unsigned char> I;
	  vpServo task;
	  vpHomogeneousMatrix wMc, wMo;
	  task.setServo(vpServo::EYEINHAND_CAMERA);
	  task.setInteractionMatrixType(vpServo::CURRENT);
	  task.setLambda(0.5);
	  vpPoint point[4];
	  point[0].setWorldCoordinates(-0.1,-0.1, 0);
	  point[1].setWorldCoordinates( 0.1,-0.1, 0);
	  point[2].setWorldCoordinates( 0.1, 0.1, 0);
	  point[3].setWorldCoordinates(-0.1, 0.1, 0);

	  vpFeaturePoint p[4], pd[4];
	  for(int i = 0; i < 4; i++){
	    point[i].track(cdMo);
	    vpFeatureBuilder::create(pd[i], point[i]);
	    point[i].track(cMo);
	    vpFeatureBuilder::create(p[i], point[i]);
	    task.addFeature(p[i], pd[i]);
	  }

	  vpImageConvert::convert(frame, I);

	  d = new vpDisplayOpenCV(I);

	  vpDisplay::display(I);

	  tracker.track(I);
	  tracker.getCameraParameters(cam);
	  tracker.display(I, cMo, cam, vpColor::red, 2);
	  vpDisplay::flush(I);

	  // The servoing
	  // TODO get position of camera from baxer set to wMc
	  // wMc starts as the 4x4 identity matrix, and is updated as we move.

	  vpSimulatorCamera robot;
      robot.setSamplingTime(0.040);
      robot.getPosition(wMc);   //  We aren't moving the robot yet, so this gets re-initialized as the identity every time.
	  wMo = wMc * cMo;
	  
	  
	  cMo = wMc.inverse() * wMo;
	  for (int i = 0; i < 4; i++){
	    point[i].track(cMo);
	    vpFeatureBuilder::create(p[i], point[i]);
	  }

	  // V is a 6 dim velocity vector
	  vpColVector v = task.computeControlLaw();
      //std::cout << v;
	  // TODO, now we have a desired velocity -> send to baxter

	  if (vpDisplay::getClick(I, false))
	    exit(0);

	}
};


int main(int argc, char** argv)
{
	if (argc==3) {
		ros::init(argc, argv, "correll_image_converter");
		char* endpoint_topic = (char*)"/robot/limb/right/endpoint_state";
		ImageConverter ic(argv[1], argv[2]);

		//	Init the tracker
		ros::spin();
		return 0;
	} else {
		std::cout<<"ERROR:\tusage - baxter-video-listener /cameras/right_hand_camera/image /robot/limb/right/endpoint_state"<<std::endl;
		return 1;
	} 
}

