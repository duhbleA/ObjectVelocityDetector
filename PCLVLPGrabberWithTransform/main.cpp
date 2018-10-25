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


//#include "../../src/transformhelper.h"

#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>

//using namespace cv;
using namespace Eigen;
//using namespace std;

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

int main( int argc, char *argv[] )
{
    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) ){
        std::cout << "usage: " << argv[0]
                  << " [-ipaddress <192.168.1.70>]"
                  << " [-port <2368>]"
                  << " [-pcap <*.pcap>]"
                  << " [-help]"
                  << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    pcl::PointCloud<PointType>::ConstPtr xformedCloud = cloud;
//    pcl::PointCloud<PointType> xformedCloud;
//    pcl::PointCloud<PointType>::ConstPtr cloud(&xformedCloud);
//    pcl::PointCloud<PointType>::ConstPtr tmp(&xformedCloud);

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Point Cloud Color Hndler
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ){
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else{
        throw std::runtime_error( "This PointType is unsupported." );
    }

//    handler->setInputCloud(cloud);

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
            [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
                boost::mutex::scoped_lock lock( mutex );

                /* Point Cloud Processing */

                pcl::transformPointCloud(*ptr, *boost::const_pointer_cast<pcl::PointCloud<PointType> >(ptr), left_rigid_body_transformation);

                cloud = ptr;
            };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    if( !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ){
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }



    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );
    InitXForms();

    // Start Grabber
    grabber->start();

    xyzw peak;

    auto last = std::chrono::steady_clock::now();

    int loopcount = 0;


    int kill = 0;
//    while(1)
//    {
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        boost::mutex::scoped_try_lock lock( mutex );
//        if( lock.owns_lock() && cloud ){
//            // Update Point Cloud
//            handler->setInputCloud( cloud );
////            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
////                viewer->addPointCloud( cloud, *handler, "cloud" );
////            }
//            size_t size = cloud->size();
//            int count = 0;
////            cout << "*****************" << endl;
//            for(const auto &z : *cloud)
//            {
////                auto v4fmap = z.getVector4fMap();

////                auto cols = v4fmap.cols();
////                auto rows = v4fmap.rows();
////                auto colstride = v4fmap.colStride();
////                auto rowstride = v4fmap.rowStride();

////                xyz dist{v4fmap.x(), v4fmap.y(), v4fmap.z()};

//                xyz dist{z._PointXYZI::x, z._PointXYZI::y, z._PointXYZI::z};

//                auto distance = dist.distance();
//                auto fAngle = dist.FrontalAngle();
////                auto vAngle = dist.VerticalAngle();


//                if((fAngle > 89.0 && fAngle < 91.0)/* && (vAngle > -5.0 && vAngle < 5.0)*/)
////                if(angle > 70.0 && angle < 110.0)
//                {
//                    int tmp = 1;
////                    if(distance < 1.0/* && distance > 0.5*/)
//                    if(distance < 0.65 && distance > 0.5)
//                    {
//                        cout << std::setprecision(4) << distance << ": (" << fAngle << ") " << endl;

////                        cout << std::setprecision(4) << distance << ": (" << fAngle << ", " << vAngle << ") " << endl;
//                    }
////                    cout << std::setprecision(4) << distance << ": (" << fAngle << ", " << vAngle << ") " << endl;

//                }

////                auto test = data[0];

//                count ++;
//            }
//            auto now = std::chrono::steady_clock::now();
//            auto msSinceLast = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
//            last = now;
////            cout << "test: " << size << ".  us since last: " << msSinceLast << endl;

//        }
//        loopcount ++;

//    }

    while( !viewer->wasStopped() )
    {
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud )
        {

            xformedCloud = cloud;
            // Update Point Cloud
//            xformedCloud = cloud;
//            xformedCloud->clear();
//            for(const auto &point : *cloud)
//            {
//                xformedCloud->push_back(point);
//                auto v4fmap = point.getVector4fMap();

//                int i = 0;
//            }
//            xformedCloud = boost::const_pointer_cast(cloud);
//            pcl::transformPointCloud(*cloud, *boost::const_pointer_cast<pcl::PointCloud<PointType> >(cloud), left_rigid_body_transformation);
//            cloud = xformedCloud;
//            rigidBodyTransform(cloud, xformedCloud, true);

            handler->setInputCloud( xformedCloud );
            if( !viewer->updatePointCloud( xformedCloud, *handler, "cloud" ) )
            {
                viewer->addPointCloud( xformedCloud, *handler, "cloud" );
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}
