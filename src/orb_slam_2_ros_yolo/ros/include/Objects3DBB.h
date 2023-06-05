#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

using namespace std;

typedef pcl::PointXYZRGBA Point7D;// 点类型 xyzrgba 点+颜色
typedef pcl::PointCloud<Point7D> PointCloud7D;// 点云类型

class Objects3DBB
{
public:
    typedef pcl::PointXYZRGBA Point7D;
    typedef pcl::PointCloud<Point7D> PointCloud7D;

    Objects3DBB ();
    ~Objects3DBB ();

    std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters); 
    void CalculateSingleObject3DBB(PointCloud7D::Ptr cloud, autosense::ObjectPtr& singleObject);
};
