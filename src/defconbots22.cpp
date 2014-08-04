#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

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

#define IMAGEVIEWER
#ifdef IMAGEVIEWER
#include <pcl/visualization/image_viewer.h>
#include <opencv2/core/core.hpp>
#endif


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


class DefconBots22
{
 public:
  typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

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
	height = 0;
	width = 0;
      }

  ~DefconBots22()
  {
  };

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
                                        //std::cout << (j*width + i)*3+0 << "," << (j*width + i)*3+1 << "," << (j*width + i)*3+2 << "," << std::endl; 
                                } 
                        }	
                        

                } 
                else 
                { cout << " --(!) No captured frame -- Break!" << endl; } 
        } 
#endif


  void 
      cloud_cb_ (const CloudConstPtr& cloud)
      {
        static double last = pcl::getTime();
        double now = pcl::getTime();
        if(now >= last + (1/FRAMES_PER_SEC)) {
          set (cloud);
          last = now;
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

	Eigen::Vector3f pointercenter (0.0f,0.0f,0.0f);
	Eigen::Vector3f pointervector (0.0f,0.0f,0.0f);


//        cout << "===============================\n"
//                "======Start of frame===========\n"
//                "===============================\n";
//        cout << "cloud size orig: " << cloud_->size() << endl;

	passx.setInputCloud (cloud_);
	passx.filter (*temp_cloud);

	passy.setInputCloud (temp_cloud);
	passy.filter (*temp_cloud2);

	passz.setInputCloud (temp_cloud2);
	passz.filter (*temp_cloud3);

//        cout << "cloud size post filter: " << temp_cloud3->size() << endl;


	if(temp_cloud3->size() > 0)
	{
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  		std::vector<pcl::PointIndices> cluster_indices;
		tree->setInputCloud (temp_cloud3);
		ec.setSearchMethod (tree);
		ec.setInputCloud (temp_cloud3);
		ec.extract (cluster_indices);
//		cout << "number of clusters " << cluster_indices.size() << endl;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			//Eigen::Vector4f centroid;
			//pcl::compute3DCentroid(*temp_cloud3, *it, centroid);

			Eigen::VectorXf centroid(6);
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

			pcl::PointXYZRGBA centroidp;
			centroidp.x = centroid[0];
			centroidp.y = centroid[1];
			centroidp.z = centroid[2];
			centroidp.r = centroid[3];
			centroidp.g = centroid[4];
			centroidp.b = centroid[5];

			cout << "centroidp " << centroidp << endl;
			cout << "centroid " << centroid << endl;

			Eigen::Vector3f centroid3f;
			centroid3f[0] = centroid[0];
			centroid3f[1] = centroid[1];
			centroid3f[2] = centroid[2];

			cout << "centroid3f " << centroid3f << endl;

			pointervector = pointercenter - centroid3f;
			cout << "distance from laser to centroid " << pointervector.norm() << endl;
			cout << "angle to unitZ " << pcl::rad2deg(acos(((pointercenter - centroid3f).normalized()).dot(Eigen::Vector3f::UnitZ()))) << endl;
			cout << "angle to unitX " << pcl::rad2deg(acos(((pointercenter - centroid3f).normalized()).dot(Eigen::Vector3f::UnitX()))) << endl;

			cout << "theta " << pcl::rad2deg(acos(pointervector[2]/pointervector.norm())) << endl;
			cout << "phi " << pcl::rad2deg(atan(pointervector[1]/pointervector[0])) << endl;
		}
	}


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
        if(filename_.empty()) {
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
	    boost::this_thread::yield ();
          }
#else

	viewerOneOff(viewer);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr emptycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(emptycloud);
        viewer.addPointCloud<pcl::PointXYZRGBA>(emptycloud, rgb, "kinect cloud");

	while ( !viewer.wasStopped () )
        {
          if (cloud_)
          {
            //the call to get() sets the cloud_ to null;
            viewer.updatePointCloud(get (), "kinect cloud");
	    viewer.spinOnce();
          }
	  boost::this_thread::yield ();
          if(!imageviewer.wasStopped()) 
	  {
		 if(width) {
        	  boost::mutex::scoped_lock lock (mtx2_);
                  imageviewer.showRGBImage(rgb_buffer,width,height); 
	  	  imageviewer.spinOnce();
		}
	  }
	  boost::this_thread::yield ();
        }
#endif

        if(filename_.empty()) {
          interface->stop ();
        }
      }


#ifndef NOVIEWER
  pcl::visualization::PCLVisualizer viewer;
#endif
#ifdef IMAGEVIEWER
  pcl::visualization::ImageViewer imageviewer;
  unsigned char rgb_buffer[sizeof (char) * 640*480*3]; 
  unsigned int height; 
  unsigned int width; 
#endif
  pcl::PassThrough<pcl::PointXYZRGBA> passx;
  pcl::PassThrough<pcl::PointXYZRGBA> passy;
  pcl::PassThrough<pcl::PointXYZRGBA> passz;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;


  pcl::PCDReader pcd;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version; 
  pcl::PCLPointCloud2::Ptr pcd_cloud;

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
