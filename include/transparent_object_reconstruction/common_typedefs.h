#ifndef TRANSP_OBJ_RECON_COMMON_DEFS
#define TRANSP_OBJ_RECON_COMMON_DEFS

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr ConstCloudPtr;

typedef pcl::PointXYZRGBL LabelPoint;
typedef pcl::PointCloud<LabelPoint> LabelCloud;
typedef LabelCloud::Ptr LabelCloudPtr;
typedef LabelCloud::ConstPtr ConstLabelCloudPtr;

static const int ANGLE_RESOLUTION = 360;
static const int OPENING_ANGLE = 20;
static const int MIN_BIN_MARKS = 120;

static const float MEDIAN_FRACTION = 1.0f;


#endif // TRANSP_OBJ_RECON_COMMON_DEFS
