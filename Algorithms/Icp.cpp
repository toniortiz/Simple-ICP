#include "Icp.h"

using namespace std;

Icp::Icp()
    : _iters(30)
    , _maxDist(0.1)
    , _epsilon(1e-9)
{
    _icp = boost::make_shared<pcl::IterativeClosestPoint<PointT, PointT>>();
    _icp->setMaximumIterations(_iters);
    _icp->setMaxCorrespondenceDistance(_maxDist);
    _icp->setTransformationEpsilon(_epsilon);

    _score = numeric_limits<double>::max();
    _Tts = Eigen::Matrix4f::Identity();
}

Icp& Icp::align(Icp::PointCloudT& src, Icp::PointCloudT& tgt)
{
    _icp->setInputSource(boost::make_shared<PointCloudT>(src));
    _icp->setInputTarget(boost::make_shared<PointCloudT>(tgt));

    _aligned.clear();
    _aligned.reserve(src.size());
    _icp->align(_aligned);

    if (_icp->hasConverged()) {
        _score = _icp->getFitnessScore();
        _Tts = _icp->getFinalTransformation();
    } else {
        _score = numeric_limits<double>::max();
        _Tts = Eigen::Matrix4f::Identity();
    }

    return *this;
}

Icp& Icp::setIterations(int iters)
{
    _iters = iters;
    _icp->setMaximumIterations(_iters);
    return *this;
}

Icp& Icp::setMaxDist(double dist)
{
    _maxDist = dist;
    _icp->setMaxCorrespondenceDistance(_maxDist);
    return *this;
}

Icp& Icp::setEpsilon(double eps)
{
    _epsilon = eps;
    _icp->setTransformationEpsilon(_epsilon);
    return *this;
}

const Eigen::Matrix4f& Icp::getTransformation() const
{
    return _Tts;
}

const double& Icp::getScore() const
{
    return _score;
}

const Icp::PointCloudT& Icp::getAligned() const
{
    return _aligned;
}

tuple<bool, const double&, const Eigen::Matrix4f&, const Icp::PointCloudT&> Icp::getResult()
{
    return { _icp->hasConverged(), _score, _Tts, _aligned };
}
