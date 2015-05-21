#ifndef TRANSP_OBJ_RECON_COMMON_DEFS
#define TRANSP_OBJ_RECON_COMMON_DEFS

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr ConstCloudPtr;

typedef pcl::PointXYZRGBL LabelPoint;
typedef pcl::PointCloud<LabelPoint> LabelCloud;
typedef LabelCloud::Ptr LabelCloudPtr;
typedef LabelCloud::ConstPtr ConstLabelCloudPtr;

#endif // TRANSP_OBJ_RECON_COMMON_DEFS
