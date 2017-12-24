#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common_headers.h>
//#include <boost/thread/thread.Hhpp>

class PCLProcessing {
    private:
    //Members
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void importOBJAsPSD(string);
    #ifdef amanDev
    string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #endif

    public:
    void viewModel();
    void viewModel(pcl::PointCloud<pcl::PointXYZ>);
    void saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ>, string);
    void saveModelAsPLY(string);
};
