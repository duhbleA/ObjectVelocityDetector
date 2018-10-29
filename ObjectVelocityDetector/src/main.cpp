#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include "../include/pythoncodecontroller.h"

#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../include/utilities.h"

#include <chrono>

using namespace cv;
using namespace Eigen;
using namespace std;

constexpr float Rad2Deg = 180.0 / M_PI;

Matrix4f left_rigid_body_transformation;
Matrix4f right_rigid_body_transformation;

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
//    left_rigid_body_transformation <<
//            0.999814f, -0.00518012f, -0.0185683f, 0.4019f,
//            0.0103731f, 0.956449f, 0.291714f, -0.31337f,
//            0.0162485f, -0.291852f, 0.956325f, -0.0502383f,
//            0, 0, 0, 1;

    right_rigid_body_transformation <<
                                    0.998874f, -0.0219703f, -0.0420543f, -0.23583f,
                                               0.0293173f, 0.982682f, 0.182967, -0.301624f,
                                               0.0373061, -0.183993f, 0.982219f, 0.0268978f,
                                               0, 0, 0, 1;
}


struct mtypes
{
    float v4;
    float v3;
    float a4;
    float a3;
};

struct xyzw
{
    mtypes x, y, z, w;
};


// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;


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

void initializePCLViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,  pcl::visualization::PointCloudColorHandler<PointType>::Ptr& handler)
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
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ) {
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

}

int main( int argc, char *argv[] )
{
    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    parseInitialArgs(argc, argv, ipaddress, port, pcap);

    // Point Clouds initialization
    pcl::PointCloud<PointType>::ConstPtr cloud;
    pcl::PointCloud<PointType>::ConstPtr xformedCloud = cloud;


    // Point Cloud Color Handler and viewer initialization
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );

    initializePCLViewer(viewer, handler);


    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
    [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ) {
        boost::mutex::scoped_lock lock( mutex );

        /* Point Cloud Processing */
        pcl::transformPointCloud(*ptr, *boost::const_pointer_cast<pcl::PointCloud<PointType> >(ptr), left_rigid_body_transformation);
        cloud = ptr;
    };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    initializeGrabber(grabber, ipaddress, port, pcap);


    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Initialize transforms
    InitXForms();

    // Initialize Python script controllers
    Py_Initialize();
    PythonCodeController* pcc = new PythonCodeController();
    pcc->start();

    // Start Grabber
    grabber->start();

    // The loop where the magic happens
    while( !viewer->wasStopped() )
    {
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock())
        {
            if (cloud)
            {
                xformedCloud = cloud;

                handler->setInputCloud( xformedCloud );
                if( !viewer->updatePointCloud( xformedCloud, *handler, "cloud" ) )
                {
                    viewer->addPointCloud( xformedCloud, *handler, "cloud" );
                }
            }

            pcc->spinOnCamera1();
            cv::Mat image = pcc->imageFromLastSpin();
            cv::Mat boxes = pcc->boxesFromLastSpin();
            cv::namedWindow("Display window 1", cv::WINDOW_AUTOSIZE);
            cv::imshow("Display window 1", image);

            if (((cv::waitKey(1) & 0xFF) == 113))
            {
                cv::destroyAllWindows();
                break;
            }
        }
    }

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
