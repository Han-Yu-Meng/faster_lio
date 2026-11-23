// Fixed-map localization extensions for lightning::LaserMapping.
// The LIO core (ESKF + ObsModel + ivox + point-to-plane ICP) is copied verbatim
// from lightning-lm; this file only adds the immutable-prior-map support.

#include "mapper_loc.h"

#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "core/lio/laser_mapping.h"

namespace lightning {

bool LaserMapping::LoadPriorMap(const std::string &pcd_path, double voxel_size,
                                const SE3 &map_to_internal) {
    PointCloudType::Ptr raw(new PointCloudType());
    if (pcl::io::loadPCDFile(pcd_path, *raw) < 0) {
        LOG(ERROR) << "[FixedMap] cannot load prior PCD: " << pcd_path;
        return false;
    }

    PointCloudType::Ptr clean(new PointCloudType());
    clean->reserve(raw->size());
    for (const auto &p : raw->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            clean->push_back(p);
        }
    }

    PointCloudType::Ptr down(new PointCloudType());
    if (voxel_size > 0.0) {
        pcl::VoxelGrid<PointType> filter;
        filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        filter.setInputCloud(clean);
        filter.filter(*down);
    } else {
        down = clean;
    }
    if (down->size() < 100) {
        LOG(ERROR) << "[FixedMap] prior has fewer than 100 valid points";
        return false;
    }

    const Mat3d R = map_to_internal.rotationMatrix();
    const Vec3d t = map_to_internal.translation();
    PointVector pts;
    pts.reserve(down->size());
    for (const auto &p : down->points) {
        Vec3d pw = R * p.getVector3fMap().cast<double>() + t;
        PointType po = p;
        po.x = static_cast<float>(pw(0));
        po.y = static_cast<float>(pw(1));
        po.z = static_cast<float>(pw(2));
        pts.push_back(po);
    }

    ivox_->AddPoints(pts);
    fixed_map_active_ = true;
    LOG(INFO) << "[FixedMap] loaded " << pts.size() << " prior points into ivox (voxel="
              << voxel_size << ")";
    return true;
}

void LaserMapping::EnableLocalizationMode() {
    options_.is_in_slam_mode_ = false;
    fixed_map_active_ = true;
}

}  // namespace lightning
