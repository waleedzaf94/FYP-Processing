
#include "pclProcessing.h"

using namespace pcl;

// Private Functions
inline 
void PCLProcessing::removePoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<int> vp, pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::IndicesPtr points (new std::vector<int>);
    points->assign(vp.begin(), vp.end());
    extract.setInputCloud (cloud);
    extract.setIndices (points);
    extract.setNegative(true);
    extract.filter(*newCloud);
}

inline
void PCLProcessing::floorFinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    std::vector<int> inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr
        model (new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ> (cloud));
    model->setAxis(Eigen::Vector3f (1.0, 0.0, 1.0));
    model->setEpsAngle(pcl::deg2rad(5.0));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
    ransac.setDistanceThreshold (0.1);
    ransac.setMaxIterations(10000);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    
    // calculate plane coefficients
    Eigen::VectorXf mc;
    mc.resize(4);
    vector<int> indices;
    for (int i=0; i<3; i++)
        indices.push_back(i);
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane (final);
    modelPlane.computeModelCoefficients(indices, mc);
    cout << "Plane coefficients: " << mc(0) << ", " << mc(1) << ", " << mc(2) << ", " << mc(3) << endl;
//    this->saveModelAsPLY(*final, this->plyFolder + "335Floor.ply");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining (new pcl::PointCloud<pcl::PointXYZ>);
    this->removePoints(cloud, inliers, remaining);
    this->saveModelAsPLY(*remaining, this->presF + "1Removed.ply");
//    this->saveModelAsPLY(*remaining, this->plyFolder + "remaining.ply");
    
    inliers.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr final2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr model2 (new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ> (remaining));
    model2->setAxis(Eigen::Vector3f (1.0, 0.0, 1.0));
    model2->setEpsAngle(pcl::deg2rad(10.0));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model2);
    ransac2.setDistanceThreshold (0.1);
    ransac2.setMaxIterations(10000);
    ransac2.computeModel();
    ransac2.getInliers(inliers);
    
    pcl::copyPointCloud<pcl::PointXYZ>(*remaining, inliers, *final2);
    
    //Plane coefficients
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane2 (final2);
    modelPlane2.computeModelCoefficients(indices, mc);
    cout << "Plane coefficients: " << mc(0) << ", " << mc(1) << ", " << mc(2) << ", " << mc(3) << endl;
    this->saveModelAsPLY(*final2, this->plyFolder + "335Roof.ply");
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining2 (new pcl::PointCloud<pcl::PointXYZ>);
    this->removePoints(remaining, inliers, remaining2);
    this->saveModelAsPLY(*remaining2, this->presF + "2Removed.ply");
//    this->saveModelAsPLY(*remaining2, this->plyFolder + "remaining2.ply");
    
    this->frRemoved = remaining2;
}

inline
void PCLProcessing::extractIndices(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_blob)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    
    // Set control variables
    float voxelGridLeafSize = 0.01;
    int segMaxIterations = 10000;
    double segDistanceThreshold = 0.15;
    double segEpsAngle = 2.0;
    double cHullAlpha = 0.3;
    
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (voxelGridLeafSize, voxelGridLeafSize, voxelGridLeafSize);
    sor.filter (*cloud_filtered);
    
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
    
    // // Write the downsampled version to disk
    pcl::PCDWriter writer;
    this->saveModelAsPLY(*cloud_filtered, this->plyFolder + "/" + this->modelFName + "_downsampled.ply" );
    this->saveModelAsPLY(*cloud_filtered, this->presF + "downsampled.ply");
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setModelType (pcl:: SACMODEL_PARALLEL_PLANE);
    // seg.setModelType (pcl:: SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (segMaxIterations);
    seg.setDistanceThreshold(segDistanceThreshold);
    
    // Settings for PERPENDICULAR PLANE
    // Vector perpendicular to the floor
    seg.setAxis(Eigen::Vector3f (0, 1, 0));
    seg.setEpsAngle(pcl::deg2rad(segEpsAngle));
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        
        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        
        // Code for concave hull
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;
        chull.setInputCloud (cloud_p);
        chull.setAlpha (cHullAlpha);
        chull.reconstruct (*cloud_hull);
        
        std::cerr << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
        
        std::stringstream ss;
        ss << this->plyFolder << "segments/" << this->modelFName << "/" << voxelGridLeafSize << "_" << segMaxIterations << "_" << segDistanceThreshold << "_" << segEpsAngle << "_" << cHullAlpha << "/";
        boost::filesystem::path dir(ss.str());
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directory(dir);
        }
        ss << i << ".ply";
        //        writer.write<pcl::PointXYZ> (ss.str (), *cloud_hull, false);
        this->saveModelAsPLY(*cloud_hull, ss.str());
        
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
}

inline
void PCLProcessing::createConcaveHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudPtr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloudPtr);
    chull.setAlpha(0.3);
    chull.reconstruct(*cloudHull);
    this->concaveHull = cloudHull;
}

// Public Members

inline
void PCLProcessing::performProcess() {
    this->saveModelAsPLY(*cloudPtr, this->presF + "pointCloud.PLY");
    this->createConcaveHull(cloudPtr);
    this->saveModelAsPLY(*concaveHull, this->presF + "concaveHull.PLY");
    this->floorFinder(concaveHull);
    this->extractIndices(frRemoved);
}

inline
void PCLProcessing::viewModel(pcl::PointCloud<pcl::PointXYZ> cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (this->cloudPtr, "one");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "one");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
}

inline
void PCLProcessing::viewModel() {    this->viewModel(this->cloud);   }

inline
void PCLProcessing::saveModelAsPLY(string filepath) {    this->saveModelAsPLY(this->cloud, filepath);    }

inline
void PCLProcessing::saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ> cloud, string filepath) {
	pcl::PLYWriter w;
	w.write(filepath, cloud);
	cout << "Saved model at " << filepath << endl;
}

inline
void PCLProcessing::importOBJAsPSD(string filename) {
	int imp = pcl::io::loadOBJFile(filename, this->cloud);
	this->cloudPtr = this->cloud.makeShared();
	if (imp == 0)
		cout << "Imported file at " << filename << endl;
}













