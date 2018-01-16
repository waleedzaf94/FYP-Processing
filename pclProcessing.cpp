//Standard
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include "pclProcessing.h"

using namespace pcl;

// Private Functions
inline
void PCLProcessing::conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr rangeCond(new pcl::ConditionAnd<pcl::PointXYZ>());
	rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
	rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("Z", pcl::ComparisonOps::LT, 0.8)));
	pcl::ConditionalRemoval<pcl::PointXYZ> cor;
	cor.setCondition(rangeCond);
	cor.setInputCloud(cloud);
	cor.setKeepOrganized(true);
	cor.filter(*filtered);
	this->saveModelAsPLY(*filtered, this->plyFolder + "corFiltered.ply");
}

inline
void PCLProcessing::planeFinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    std::vector<int> inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    this->saveModelAsPLY(*final, this->plyFolder + "/initPlane.ply");
}


inline 
void PCLProcessing::removePoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<int> points, pcl::PointCloud<pcl::PointXYZ>::ConstPtr newCloud)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
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
    model->setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
    model->setEpsAngle(pcl::deg2rad(15.0));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    
    Eigen::VectorXf mc;
    mc.resize(4);
    vector<int> indices;
    for (int i=0; i<3; i++)
        indices.push_back(i);
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane (final);
    modelPlane.computeModelCoefficients(indices, mc);
    cout << "Plane coefficients: " << mc(0) << ", " << mc(1) << ", " << mc(2) << ", " << mc(3) << endl;
    
    this->saveModelAsPLY(*final, this->plyFolder + "/z15.ply");
}

inline
void PCLProcessing::wallFinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    std::vector<int> inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud, false));
    model->setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
    model->setEpsAngle (pcl::deg2rad (5.0));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model, 0.01);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    
    Eigen::VectorXf mc;
    mc.resize(4);
    vector<int> indices;
    for (int i=0; i<3; i++)
        indices.push_back(i);
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane (final);
    modelPlane.computeModelCoefficients(indices, mc);
    cout << "Plane coefficients: " << mc(0) << ", " << mc(1) << ", " << mc(2) << ", " << mc(3) << endl;
//    model->computeModelCoefficients(indices, mc);

    this->saveModelAsPLY(*final, this->plyFolder + "wall.ply");
}
inline
void PCLProcessing::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
	ror.setInputCloud(cloud);
	ror.setRadiusSearch(0.8);
	ror.setMinNeighborsInRadius(2);
	ror.filter(*filtered);
	this->saveModelAsPLY(*filtered, this->plyFolder + "rorFiltered.ply");
}

inline
void PCLProcessing::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(false);
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*filtered);
	this->saveModelAsPLY(*filtered, this->plyFolder + "sorFiltered.ply");
	cout << "Completed sor." << endl;
}

inline
void PCLProcessing::getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    
}

inline
void PCLProcessing::diffOfNormalsSegmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	//The smallest scale to use in the DoN filter.
    double scale1 = 0.1;

    ///The largest scale to use in the DoN filter.
    double scale2 = 10.0;

    ///The minimum DoN magnitude to threshold by
    double threshold = 0.2;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius = 5.0;

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (cloud->isOrganized ()) {
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    }
    else
    {
        tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (cloud);

    if (scale1 >= scale2)
    {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
    }
    std::cout << "Pointcloud: " << cloud->points.size () << " data points." << std::endl;
    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);

    /**
    * NOTE: setting viewpoint is very important, so that we can ensure
    * normals are all pointed in the same direction!
    */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scale2 << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Save DoN features
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

    // Filter by magnitude
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<pcl::PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                               new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
    // Build the filter

    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition(range_cond);

    condrem.setInputCloud (doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

    writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);

    // Filter by magnitude
    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        stringstream ss;
        ss << "don_cluster_" << j << ".pcd";
        writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    }
}

// Public Members
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



inline
void PCLProcessing::performProcess()
{
    this->floorFinder(cloudPtr);
//    this->wallFinder(cloudPtr);
}
