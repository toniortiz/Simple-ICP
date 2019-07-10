#ifndef ABSTRACTICP_H
#define ABSTRACTICP_H

#include <Eigen/Dense>
#include <pcl/registration/registration.h>
#include <tuple>

class AbstractIcp {
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

public:
    AbstractIcp();

    virtual AbstractIcp& align(PointCloudT& src, PointCloudT& tgt) = 0;

    AbstractIcp& setIterations(int iters);
    AbstractIcp& setMaxDist(double dist);
    AbstractIcp& setEpsilon(double eps);

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
    pcl::Registration<PointT, PointT>::Ptr _reg;

    // Results
    double _score;
    PointCloudT _aligned;
    Eigen::Matrix4f _Tts; // Transform points from source to target cloud
};

#endif // ABSTRACTICP_H
