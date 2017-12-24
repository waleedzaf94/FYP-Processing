//Standard
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include "pclProcessing.h"

// Private Functions
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


void PCLProcessing::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.8);
    ror.setMinNeighborsInRadius(2);
    ror.filter(*filtered);
    this->saveModelAsPLY(*filtered, this->plyFolder + "rorFiltered.ply");
}
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

void PCLProcessing::importOBJAsPSD(string filename) {
    int imp = pcl::io::loadOBJFile(filename, this->cloud);
    this->cloudPtr = this->cloud.makeShared();
    if (imp == 0)
        cout << "Imported file at " << filename << endl;
}

// Public Members
void PCLProcessing::viewModel(pcl::PointCloud<pcl::PointXYZ> cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (this->cloudPtr, "one");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "one");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
}
void PCLProcessing::viewModel() {    this->viewModel(this->cloud);   }
void PCLProcessing::saveModelAsPLY(string filepath) {    this->saveModelAsPLY(this->cloud, filepath);    }
void PCLProcessing::saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ> cloud, string filepath) {
    pcl::PLYWriter w;
    w.write(filepath, cloud);
    cout << "Saved model at " << filepath << endl;
}
