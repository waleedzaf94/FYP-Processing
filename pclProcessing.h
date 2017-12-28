#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

//#include <boost/thread/thread.Hhpp>

class PCLProcessing {
    private:
    //Membersc
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
    #ifdef amanDev
    string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #endif
    // functions
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void diffOfNormalsSegmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    public:
    void viewModel();
    void viewModel(pcl::PointCloud<pcl::PointXYZ>);
    void saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ>, string);
    void saveModelAsPLY(string);
    void performProcess();
    void importOBJAsPSD(string);
};
