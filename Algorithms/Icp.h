#ifndef ICP_H
#define ICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tuple>

class Icp {
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

public:
    Icp();

    Icp& align(PointCloudT& src, PointCloudT& tgt);

    Icp& setIterations(int iters);
    Icp& setMaxDist(double dist);
    Icp& setEpsilon(double eps);

    const Eigen::Matrix4f& getTransformation() const;
    const double& getScore() const;
    const PointCloudT& getAligned() const;

    std::tuple<bool, const double&, const Eigen::Matrix4f&, const PointCloudT&> getResult();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    // Parameters
    int _iters;
    double _maxDist;
    double _epsilon;
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr _icp;

    // Results
    double _score;
    PointCloudT _aligned;
    Eigen::Matrix4f _Tts; // Transform points from source to target cloud
};

#endif // ICP_H
