// lightning's LIO core uses the custom PointXYZIT type with PCL templates.
// PCL's prebuilt libraries only instantiate standard point types, so these
// symbols must be emitted explicitly into this plugin (otherwise dlopen fails
// with "undefined symbol: pcl::PCLBase<PointXYZIT>::setInputCloud").
#include <pcl/filters/voxel_grid.h>

#include "common/point_def.h"

template class pcl::PCLBase<PointXYZIT>;
template class pcl::VoxelGrid<PointXYZIT>;
