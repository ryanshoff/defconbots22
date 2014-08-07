#define IMAGEVIEWER
#define TRACKVIEWER
#undef SERIAL
#undef NOVIEWER


#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <SerialStream.h>

#ifndef NOVIEWER
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <defconbots22.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/conversions.h>

#include <pcl/visualization/image_viewer.h>
#include <opencv2/core/core.hpp>

class DefconBots22
{
    public:
        typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
        typedef typename Cloud::Ptr CloudPtr;
        typedef typename Cloud::ConstPtr CloudConstPtr;


#ifndef NOVIEWER
        void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
        {
            pcl::PointXYZ  origin(0.0, 0.0, 0.0);
            pcl::PointCloud<pcl::PointXYZRGBA> boundingbox;
            pcl::PointXYZRGBA point;

            viewer.addText3D("DefconBots", origin, .1);

            point.x = MIN_X_DEPTH;
            point.y = MIN_Y_DEPTH;
            point.z = MIN_Z_DEPTH;
            point.r = 255;
            point.g = 255;
            point.b = 255;

            boundingbox.points.push_back(point);
            point.x = MAX_X_DEPTH;
            boundingbox.points.push_back(point);
            point.y = MAX_Y_DEPTH;
            boundingbox.points.push_back(point);
            point.x = MIN_X_DEPTH;
            boundingbox.points.push_back(point);
            point.z = MAX_Z_DEPTH;
            point.y = MIN_Y_DEPTH;
            boundingbox.points.push_back(point);
            point.x = MAX_X_DEPTH;
            boundingbox.points.push_back(point);
            point.y = MAX_Y_DEPTH;
            boundingbox.points.push_back(point);
            point.x = MIN_X_DEPTH;
            boundingbox.points.push_back(point);

            viewer.addLine(boundingbox[0],boundingbox[1], "1", 0);
            viewer.addLine(boundingbox[1],boundingbox[2], "2", 0);
            viewer.addLine(boundingbox[2],boundingbox[3], "3", 0);
            viewer.addLine(boundingbox[3],boundingbox[0], "4", 0);
            viewer.addLine(boundingbox[4],boundingbox[5], "5", 0);
            viewer.addLine(boundingbox[5],boundingbox[6], "6", 0);
            viewer.addLine(boundingbox[6],boundingbox[7], "7", 0);
            viewer.addLine(boundingbox[7],boundingbox[4], "8", 0);
            viewer.addLine(boundingbox[0],boundingbox[4], "9", 0);
            viewer.addLine(boundingbox[1],boundingbox[5], "10", 0);
            viewer.addLine(boundingbox[2],boundingbox[6], "11", 0);
            viewer.addLine(boundingbox[3],boundingbox[7], "12", 0);

            viewer.initCameraParameters();
            viewer.resetCameraViewpoint();
        }

#endif


        DefconBots22 (const std::string& device_id = "", const std::string& filename = "" ) :
#ifndef NOVIEWER
            viewer ("defconbots22"),
#endif
            device_id_ (device_id) , filename_ (filename)
            {
                passx.setFilterFieldName("x");
                passx.setFilterLimits(MIN_X_DEPTH, MAX_X_DEPTH);

                passy.setFilterFieldName("y");
                passy.setFilterLimits(MIN_Y_DEPTH, MAX_Y_DEPTH);

                passz.setFilterFieldName("z");
                passz.setFilterLimits(MIN_Z_DEPTH, MAX_Z_DEPTH);

                ec.setClusterTolerance (0.02); // 2cm
                ec.setMinClusterSize (50);
                ec.setMaxClusterSize (25000);


#ifndef NOVIEWER
                viewer.registerKeyboardCallback(&DefconBots22::keyboard_callback, *this , 0);
#endif
                saveCloud = false;
                toggleView = 0;
                filesSaved = 0;
#ifdef IMAGEVIEWER
                height = 0;
                width = 0;
#endif

                openserial();
                bzero(rgb_track,sizeof (char) * TRACKSIZE*TRACKSIZE*3); 
            }

        ~DefconBots22()
        {
        };


        void
            openserial()
            {


#ifdef SERIAL
                using namespace LibSerial ;
                serial_port.Open( SERIAL_PORT_DEVICE ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not open serial port "
                        << SERIAL_PORT_DEVICE
                        << std::endl ;
                    exit(1) ;
                }
                //
                // Set the baud rate of the serial port.
                //
                serial_port.SetBaudRate( SerialStreamBuf::BAUD_57600 ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not set the baud rate." << std::endl ;
                    exit(1) ;
                }
                //
                // Set the number of data bits.
                //
                serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not set the character size." << std::endl ;
                    exit(1) ;
                }
                //
                // Disable parity.
                //
                serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not disable the parity." << std::endl ;
                    exit(1) ;
                }
                //
                // Set the number of stop bits.
                //
                serial_port.SetNumOfStopBits( 1 ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not set the number of stop bits."
                        << std::endl ;
                    exit(1) ;
                }
                //
                // Turn on hardware flow control.
                //
                serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
                if ( ! serial_port.good() )
                {
                    std::cerr << "Error: Could not use hardware flow control."
                        << std::endl ;
                    exit(1) ;
                }
                //
                // Do not skip whitespace characters while reading from the
                // serial port.
                //
                // serial_port.unsetf( std::ios_base::skipws ) ;

#endif
            }


        void
            keyboard_callback (const pcl::visualization::KeyboardEvent& event, void *)
            {
                if (event.keyUp ())
                {
                    switch (event.getKeyCode())
                    {
                        case 's':
                        case 'S':
                            saveCloud = true; // save pcd file
                            break;
                        case 't':
                        case 'T':
                            ++toggleView  %= 2; 
                            break;
                        case 'c':
                        case 'C':
                            bzero(rgb_track,sizeof (char) * TRACKSIZE*TRACKSIZE*3); 
                            break;
                    }
                }	
            }

#ifdef IMAGEVIEWER
        void image_cb_ (const boost::shared_ptr<openni_wrapper::Image> &image) 
        { 
            boost::mutex::scoped_lock lock (mtx2_);
            height = image->getHeight(); 
            width = image->getWidth(); 
            cv::Mat frameBGR=cv::Mat(image->getHeight(),image->getWidth(),CV_8UC3); 

            image->fillRGB(frameBGR.cols,frameBGR.rows,frameBGR.data,frameBGR.step); 

            //-- 3. Apply the classifier to the frame 
            if( !frameBGR.empty() ) 
            { 
                for(int j=0; j<height; j++) 
                { 
                    for(int i=0; i<width; i++) 
                    { 
                        rgb_buffer[(j*width + i)*3+0] = frameBGR.at<cv::Vec3b>(j,i)[0];  // B 
                        rgb_buffer[(j*width + i)*3+1] = frameBGR.at<cv::Vec3b>(j,i)[1];  // G 
                        rgb_buffer[(j*width + i)*3+2] = frameBGR.at<cv::Vec3b>(j,i)[2];  // R 
                        //cout << (j*width + i)*3+0 << "," << (j*width + i)*3+1 << "," << (j*width + i)*3+2 << "," << std::endl; 
                    } 
                }	


            } 
            else 
            { 
                //cout << " --(!) No captured frame -- Break!" << endl; 
            } 
        } 
#endif


        void 
            cloud_cb_ (const CloudConstPtr& cloud)
            {
                static unsigned count = 0;
                static double last = pcl::getTime();
                double now = pcl::getTime();
                if(++count == FRAMES_PER_SEC) {
                    set (cloud);
                    last = now;
                    count = 0;
                }
            }

        void
            set (const CloudConstPtr& cloud)
            {
                //lock while we set our cloud;
                boost::mutex::scoped_lock lock (mtx_);
                cloud_  = cloud;
            }

        CloudPtr
            get ()
            {
                //lock while we swap our cloud and reset it.
                boost::mutex::scoped_lock lock (mtx_);
                CloudPtr temp_cloud (new Cloud);
                CloudPtr temp_cloud2 (new Cloud);
                CloudPtr temp_cloud3 (new Cloud);
                CloudPtr temp_cloud4 (new Cloud);
                CloudPtr temp_cloud5 (new Cloud);
                CloudConstPtr empty_cloud;

                Eigen::Vector3f pointercenter (0.0f,0.1f,0.0f);
                Eigen::Vector3f pointervector (0.0f,0.0f,0.0f);


                cout << "===============================\n"
                    "======Start of frame===========\n"
                    "===============================\n";
                cout << "cloud size orig: " << cloud_->size() << endl;

                passx.setInputCloud (cloud_);
                passx.filter (*temp_cloud);

                passy.setInputCloud (temp_cloud);
                passy.filter (*temp_cloud2);

                passz.setInputCloud (temp_cloud2);
                passz.filter (*temp_cloud3);

                cout << "cloud size post filter: " << temp_cloud3->size() << endl;


                centroid_vector.clear();

                if(temp_cloud3->size() > 0)
                {
                    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
                    std::vector<pcl::PointIndices> cluster_indices;
                    tree->setInputCloud (temp_cloud3);
                    ec.setSearchMethod (tree);
                    ec.setInputCloud (temp_cloud3);
                    ec.extract (cluster_indices);
                    cout << "number of clusters " << cluster_indices.size() << endl;


                    unsigned int count = 0;
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                    {
                        count++;
                        cout << "centroid " << count << endl;
                        Eigen::VectorXf centroid(8);

                        centroid.setZero();
                        const std::vector<int> indices = (*it).indices;
                        for(size_t i = 0; i < indices.size(); i++)
                        {
                            centroid[0] += (*temp_cloud3)[indices[i]].x;
                            centroid[1] += (*temp_cloud3)[indices[i]].y;
                            centroid[2] += (*temp_cloud3)[indices[i]].z;
                            centroid[3] += (*temp_cloud3)[indices[i]].r;
                            centroid[4] += (*temp_cloud3)[indices[i]].g;
                            centroid[5] += (*temp_cloud3)[indices[i]].b;
                        }
                        centroid /= indices.size();


                        Eigen::Vector3f centroid3f;
                        centroid3f[0] = centroid[0];
                        centroid3f[1] = centroid[1];
                        centroid3f[2] = centroid[2];

                        pointervector = pointercenter - centroid3f;

                        float theta = pcl::rad2deg(atan(centroid[0]/centroid[2]));
                        unsigned int thetapwm = theta * 10 + 1200;

                        float phi = pcl::rad2deg(atan(-centroid[1]/centroid[2]));
                        unsigned int phipwm = phi * 10 + 1340;

                        float thetapointer = pcl::rad2deg(atan(pointervector[0]/pointervector[2]));
                        float phipointer = pcl::rad2deg(atan(-pointervector[1]/pointervector[2]));

                        centroid[6] = phipwm;
                        centroid[7] = thetapwm;

                        unsigned int x = centroid[0]*100 + TRACKSIZE/2;
                        unsigned int z = TRACKSIZE - centroid[2]*100;
                        rgb_track[(z*TRACKSIZE + x)*3+0] = 255;  // R 
                        rgb_track[(z*TRACKSIZE + x+1)*3+0] = 255;  // R 
                        rgb_track[((z+1)*TRACKSIZE + x)*3+0] = 255;  // R 
                        rgb_track[((z+1)*TRACKSIZE + x+1)*3+0] = 255;  // R 

                        cout << centroid << endl;
                        cout << "distance from laser to centroid " << pointervector.norm() << " meters." << endl;

                        cout << "theta angle " << theta << " degrees.  phi angle " << phi << " degrees." << endl;
                        cout << "theta pointer angle " << thetapointer << " degrees.  phi angle " << phipointer << " degrees." << endl;
                        cout << phipwm  << "," << thetapwm << ",10" << endl;

                        centroid_vector.push_back(centroid);
                    }
                }
                // need mutex here
                centroid_vector_.swap(centroid_vector);


                if (saveCloud)
                {
                    std::stringstream stream, stream1;
                    std::string filename;

                    stream << "inputCloud" << filesSaved << ".pcd";
                    filename = stream.str();
                    if (pcl::io::savePCDFile(filename, *cloud_, true) == 0)
                    {
                        filesSaved++;
                        cout << "Saved " << filename << "." << endl;
                    }
                    else PCL_ERROR("Problem saving %s.\n", filename.c_str());


                    stream1 << "inputCloud" << filesSaved << ".pcd";
                    filename = stream1.str();
                    if (pcl::io::savePCDFile(filename, *temp_cloud5, true) == 0)
                    {
                        filesSaved++;
                        cout << "Saved " << filename << "." << endl;
                    }
                    else PCL_ERROR("Problem saving %s.\n", filename.c_str());

                    saveCloud = false;
                }

                empty_cloud.swap(cloud_);  // set cloud_ to null

                if(toggleView == 1) 
                    return (temp_cloud);  // return zcloud
                else
                    return (temp_cloud3); // return xyzcloud
            }

        void
            run ()
            {
                CloudPtr filecloud;
                pcl::Grabber* interface;
                if(filename_.empty())   
                {
                    interface = new pcl::OpenNIGrabber (device_id_);

                    boost::function<void (const CloudConstPtr&)> f = boost::bind (&DefconBots22::cloud_cb_, this, _1);
                    boost::signals2::connection c = interface->registerCallback (f);

#ifdef IMAGEVIEWER
                    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> g = 
                        boost::bind (&DefconBots22::image_cb_, this, _1); 

                    boost::signals2::connection d = interface->registerCallback (g);
#endif

                    interface->start ();
                }
                else 
                {
                    pcd_cloud.reset (new pcl::PCLPointCloud2);
                    if(pcd.read (filename_, *pcd_cloud, origin, orientation, version) < 0)
                        cout << "file read failed" << endl;
                    filecloud.reset (new Cloud);
                    pcl::fromPCLPointCloud2(*pcd_cloud, *filecloud);
                    cloud_  = filecloud;
                }


#ifdef NOVIEWER        
                if(!filename_.empty()) 
                    get();
                else 
                    while(TRUE)
                    {
                        if (cloud_)
                            get();
                        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
                    }
#else

                viewerOneOff(viewer);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr emptycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(emptycloud);
                viewer.addPointCloud<pcl::PointXYZRGBA>(emptycloud, rgb, "kinect cloud");

                unsigned int count = 0;
                while ( !viewer.wasStopped () )
                {
                    if (cloud_)
                    {
                        //the call to get() sets the cloud_ to null;
                        viewer.updatePointCloud(get (), "kinect cloud");
#ifdef SERIAL
                        if(!centroid_vector_.empty())
                            serial_port << (unsigned int) (centroid_vector_[0])[6] << "," << (unsigned int) (centroid_vector_[0])[7] << ",10" << endl;
#endif
                        viewer.spinOnce();
                        count++;
                        if(count%2)
                        {
                            //cout << "1000,1000,10" << endl;
                            //serial_port << "1000,1000,10" << endl;
                        }
                    }

#ifdef IMAGEVIEWER
                    if(!imageviewer.wasStopped()) 
                    {
                        if(width) {
                            updateImage();
                            imageviewer.spinOnce();
                        }
                    }
#endif
#ifdef TRACKVIEWER
                    if(!imageviewertrack.wasStopped()) 
                    {
                        imageviewertrack.showRGBImage(rgb_track,TRACKSIZE,TRACKSIZE); 
                        imageviewertrack.spinOnce();
                    }
#endif
                    boost::this_thread::sleep (boost::posix_time::microseconds (10000));

                    if(!filename_.empty())
                        viewer.spinOnce();
                }
#endif

                if(filename_.empty()) {
                    interface->stop ();
                }
            }

#ifdef IMAGEVIEWER
        void
            updateImage() {
                boost::mutex::scoped_lock lock (mtx2_);
                imageviewer.showRGBImage(rgb_buffer,width,height); 
                width = 0;
            }
#endif


#ifndef NOVIEWER
        pcl::visualization::PCLVisualizer viewer;
#endif
#ifdef IMAGEVIEWER
        pcl::visualization::ImageViewer imageviewer;
        unsigned char rgb_buffer[sizeof (char) * 640*480*3]; 
        unsigned int height; 
        unsigned int width; 
#endif

#ifdef TRACKVIEWER
        pcl::visualization::ImageViewer imageviewertrack;
#endif
        unsigned char rgb_track[sizeof (char) * TRACKSIZE*TRACKSIZE*3]; 

        pcl::PassThrough<pcl::PointXYZRGBA> passx;
        pcl::PassThrough<pcl::PointXYZRGBA> passy;
        pcl::PassThrough<pcl::PointXYZRGBA> passz;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;

        LibSerial::SerialStream serial_port;

        pcl::PCDReader pcd;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int version; 
        pcl::PCLPointCloud2::Ptr pcd_cloud;
        std::vector<Eigen::VectorXf> centroid_vector_;
        std::vector<Eigen::VectorXf> centroid_vector;

        std::string device_id_;
        std::string filename_;
        boost::mutex mtx_;
        boost::mutex mtx2_;
        CloudConstPtr cloud_;
        bool saveCloud; 
        unsigned int toggleView;
        unsigned int filesSaved;
};


    void
usage (char ** argv)
{
    std::cout << "usage: " << argv[0] << " <-device device_id> <-file filename.pcd>\n\n";

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () > 0)
    {
        for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
        {
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
                << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
            cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
                << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
                << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
        }
    }
    else
        cout << "No devices connected." << endl;
}

    int 
main (int argc, char ** argv)
{
    std::string filename;
    std::string device;

    if ( pcl::console::find_switch(argc, argv, "--help") || 
            pcl::console::find_switch(argc, argv, "-h"))
    {
        usage (argv);
        return 1;
    }

    //double threshold = 0.05;
    //pcl::console::parse_argument (argc, argv, "-thresh", threshold);

    pcl::console::parse_argument (argc, argv, "-file", filename);

    if(filename.empty() == FALSE)
        cout << "filename: " << filename << endl;

    pcl::console::parse_argument (argc, argv, "-device", device);

    if(device.empty() == TRUE)
        device += "#1";

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () == 0 && filename.empty()) 
    {
        cout << "No devices connected. No filename." << endl;
        return 1;
    }

    DefconBots22 v (device, filename);
    v.run ();

    return (0);
}
