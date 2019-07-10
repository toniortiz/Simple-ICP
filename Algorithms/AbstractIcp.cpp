#include "AbstractIcp.h"

using namespace std;

AbstractIcp::AbstractIcp()
    : _iters(30)
    , _maxDist(0.1)
    , _epsilon(1e-9)
{
    _score = numeric_limits<double>::max();
    _Tts = Eigen::Matrix4f::Identity();
}

AbstractIcp& AbstractIcp::setIterations(int iters)
{
    _iters = iters;
    _reg->setMaximumIterations(_iters);
    return *this;
}

AbstractIcp& AbstractIcp::setMaxDist(double dist)
{
    _maxDist = dist;
    _reg->setMaxCorrespondenceDistance(_maxDist);
    return *this;
}

AbstractIcp& AbstractIcp::setEpsilon(double eps)
{
    _epsilon = eps;
    _reg->setTransformationEpsilon(_epsilon);
    return *this;
}

const Eigen::Matrix4f& AbstractIcp::getTransformation() const
{
    return _Tts;
}

const double& AbstractIcp::getScore() const
{
    return _score;
}

const AbstractIcp::PointCloudT& AbstractIcp::getAligned() const
{
    return _aligned;
}

tuple<bool, const double&, const Eigen::Matrix4f&, const AbstractIcp::PointCloudT&> AbstractIcp::getResult()
{
    return { _reg->hasConverged(), _score, _Tts, _aligned };
}
