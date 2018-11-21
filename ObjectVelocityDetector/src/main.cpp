#include "../include/pclutils.h"
#include "../include/mathutils.h"
#include "../include/pythoncodecontroller.h"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>


using namespace cv;
using namespace Eigen;
using namespace std;


Matrix4f left_rigid_body_transformation;
Matrix4f right_rigid_body_transformation;

cv::Mat left_projection_matrix;

float min_point, max_point;

// Point Clouds initialization
pcl::PointCloud<PointType>::ConstPtr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::ConstPtr modifiedCloud(new pcl::PointCloud<PointType>);


cv::Rect frame;
cv::Mat boxes;


// Retrieved Point Cloud Callback Function
std::mutex sys_mutex;
std::vector<PointType> visiblePoints;

std::vector<Sample> allSamples;
std::vector<Sample> samplesToRender;

PythonCodeController* pcc;


cv::Mat image;

bool ready = false;
bool running = true;




void InitXForms()
{
//    left_rigid_body_transformation <<
//            1.0f,       -0.0f,          -0.0f,      0.4019f,
//            0.0f,       1.0f,           0.291714f,  -0.31337f,
//            0.0f,       -0.291852f,     1.0f,       -0.0f,
//            0,          0,              0,          1.0f;


    left_rigid_body_transformation <<
                                   1.0f,       -0.0f,          -0.0f,      0.31337f,
                                               0.0f,       1.0f,           0.0,        -0.0f,
                                               0.0f,       -0.0,           1.0f,       -0.0f,
                                               0,          0,              0,          1.0f;
    //left_rigid_body_transformation <<
    //0.999814f, -0.00518012f, -0.0185683f, 0.4019f,
    //0.0103731f, 0.956449f, 0.291714f, -0.31337f,
    //0.0162485f, -0.291852f, 0.956325f, -0.0502383f,
    //0, 0, 0, 1;

    right_rigid_body_transformation <<
                                    0.998874f, -0.0219703f, -0.0420543f, -0.23583f,
                                               0.0293173f, 0.982682f, 0.182967, -0.301624f,
                                               0.0373061, -0.183993f, 0.982219f, 0.0268978f,
                                               0, 0, 0, 1;

    /// old 1920x1080 camera view
//    float left_raw_projection[12] = {942.129700f,   0.0,            985.634114f,    0.0,
//                                     0.0,           1060.674438f,   600.441036f,    0.0,
//                                     0.0,           0.0,            1.0f,           0.0
//                                    };

    /// Re-centered but not quite right
//    float left_raw_projection[12] = {702.129700f,   0.0,            745.634114f,    0.0,
//                                     0.0,           740.674438f,    280.441036f,    0.0,
//                                     0.0,           0.0,            1.0f,           0.0
//                                    };
    /// adjusting to the 1280x720 view
    float left_raw_projection[12] = {1182.129700f,  0.0,            750.634114f,    0.0,
                                     0.0,           1380.674438f,   350.441036f,    0.0,
                                     0.0,           0.0,            1.0f,           0.0
                                    };


    float left_raw_projection_example[12] = {611.651245f, 0.0f, 642.388357f, 0.0f,
                                             0.0f, 688.443726f, 365.971718f, 0.0f,
                                             0.0f, 0.0f, 1.0f, 0.0f
                                            };

    cv::Mat(3, 4, CV_32FC1, &left_raw_projection).copyTo(left_projection_matrix);
}


void parseInitialArgs(int argc, char *argv[], std::string& ipaddress, std::string& port, std::string& pcap)
{
    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) ) {
        std::cout << "usage: " << argv[0]
                  << " [-ipaddress <192.168.1.70>]"
                  << " [-port <2368>]"
                  << " [-pcap <*.pcap>]"
                  << " [-help]"
                  << std::endl;
        exit(1);
    }

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;
}

void initializePCLHandler(pcl::visualization::PointCloudColorHandler<PointType>::Ptr& handler)
{
    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ) {
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ) {
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ) {
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else {
        throw std::runtime_error( "This PointType is unsupported." );
    }
}

void initializePCLViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,  pcl::visualization::PointCloudColorHandler<PointType>::Ptr& handler)
{
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ) {
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ) {
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ) {
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else {
        throw std::runtime_error( "This PointType is unsupported." );
    }
}

void initializeGrabber(boost::shared_ptr<pcl::VLPGrabber>& grabber, std::string& ipaddress, std::string& port, std::string& pcap)
{
    if( !pcap.empty() ) {
        std::cout << "Capturing from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ) {
        std::cout << "Capturing from VLP-16..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

}


void Function()
{
    while(running)
    {   
        ready = false;
        if(sys_mutex.try_lock())
        {
            boost::const_pointer_cast<pcl::PointCloud<PointType>>(modifiedCloud)->points.clear();
            for(const PointType &point : cloud->points)
            {
                boost::const_pointer_cast<pcl::PointCloud<PointType>>(modifiedCloud)->push_back(point);
            }

            sys_mutex.unlock();
            ready = true;
        }

        if (ready)
        {
            std::vector<PointType> tmpPoints;
            bool RenderBoxes = true;
            visiblePoints.clear();
            samplesToRender.clear();


            if (modifiedCloud)
            {
                if(RenderBoxes)
                {
                    TrimPoints(modifiedCloud, frame, left_projection_matrix, tmpPoints, min_point, max_point);
                    FilterBoundingBox(tmpPoints, left_projection_matrix, boxes, visiblePoints, -1.0, allSamples, samplesToRender);
                }
                else
                {
                    TrimPoints(modifiedCloud, frame, left_projection_matrix, visiblePoints, min_point, max_point);
                }

                pcc->spinOnCamera1();
                image = pcc->imageFromLastSpin();
                boxes = pcc->boxesFromLastSpin();

                frame = cv::Rect(0, 0, image.cols, image.rows);

                project(left_projection_matrix, frame, image, visiblePoints, min_point, max_point);

                std::cout << "___Object Velocities (m/s)___" << std::endl;
                int i = 0;
                for (Sample &sample : samplesToRender)
                {
                    std::cout << "Object: " << i++ << "\n";
                    std::cout << "\tCount: " << std::to_string(sample.pointsInside) << "\n";
                    std::cout << "\tRunning: " << std::to_string(sample.runningDistance) << "\n";
                    std::cout << "\tStd_Dev: " << std::to_string(sample.std_dev) << "\n";
                    std::cout << "\t" << std::to_string(sample.velocity) << std::endl;
                    cv::putText(image, std::to_string(sample.velocity) + " m/s", sample.boundingBox.br(), cv::FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,255,255), 2);
                }

                cv::namedWindow("Display window 1", cv::WINDOW_AUTOSIZE);
                cv::imshow("Display window 1", image);


                for (Sample &sample : allSamples)
                {
                    sample.framesSinceLastSeen += 1;
                }

                allSamples.erase(std::remove_if(allSamples.begin(), allSamples.end(), [](const Sample & sample) { return sample.framesSinceLastSeen > 10;}), allSamples.end());

            }
        }
    }
}

int main( int argc, char *argv[] )
{
    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    parseInitialArgs(argc, argv, ipaddress, port, pcap);

    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
    []( const pcl::PointCloud<PointType>::ConstPtr& ptr ) {

            sys_mutex.lock();
            /* Point Cloud Processing */
            performTransform(*ptr, *boost::const_pointer_cast<pcl::PointCloud<PointType> >(ptr), 0, 0, 0,  M_PI / 2, 0, 0);

            pcl::transformPointCloud(*ptr, *boost::const_pointer_cast<pcl::PointCloud<PointType> >(ptr), left_rigid_body_transformation);

            cloud = ptr;
            sys_mutex.unlock();
    };


    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    initializeGrabber(grabber, ipaddress, port, pcap);


    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Initialize transforms
    InitXForms();

    // Initialize Python script controllers
    std::cout << "Initializing python" << std::endl;
    Py_Initialize();
    try
    {
        pcc = new PythonCodeController();
    }
    catch(std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    std::cout << "Starting camera drivers" << std::endl;
    pcc->start();

    std::cout << "Camera drivers started" << std::endl;

    // Start Grabber
    grabber->start();

    std::thread th = std::thread(&Function /*, this*/);

    // The loop where the magic happens
    while( 1 )
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (((cv::waitKey(1) & 0xFF) == 113))
        {
            cv::destroyAllWindows();
            break;
        }
    }

      running = false;
      th.join();

    // Stop Python script controller
    pcc->terminate();
    Py_Finalize();

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ) {
        connection.disconnect();
    }

    return 0;
}
