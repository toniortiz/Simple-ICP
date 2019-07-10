#include "Viewer.h"
#include "Utils/Random.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;

Viewer::Viewer(const string& name)
    : _finishRequested(false)
    , _finished(true)
    , _name(name)
    , _pointSize(2)
{
}

Viewer::PointCloudT Viewer::open(const string& filename, const bool& render)
{
    PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();

    string ext = filename.substr(filename.find_last_of(".") + 1);
    if (ext == "pcd")
        pcl::io::loadPCDFile(filename, *cloud);
    else if (ext == "ply")
        pcl::io::loadPLYFile(filename, *cloud);

    if (render) {
        unique_lock<mutex> lock(_mutexViewer);

        _colors.push_back(boost::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointT>>(cloud,
            Random::randomInt(0, 255), Random::randomInt(0, 255), Random::randomInt(0, 255)));
        _clouds.push_back(cloud);
        _cloudNames.push_back(filename);
        _inserted.push_back(false);
    }

    return *cloud;
}

void Viewer::add(const PointCloudT& cloud, const string& name)
{
    unique_lock<mutex> lock(_mutexViewer);
    PointCloudT::Ptr cloud_ptr(new PointCloudT(cloud));
    _colors.push_back(boost::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointT>>(cloud_ptr,
        Random::randomInt(0, 255), Random::randomInt(0, 255), Random::randomInt(0, 255)));
    _clouds.push_back(cloud_ptr);
    _cloudNames.push_back(name);
    _inserted.push_back(false);
}

Viewer& Viewer::start()
{
    if (!_thread.joinable())
        _thread = thread(&Viewer::run, this);

    return *this;
}

void Viewer::run()
{
    _finished = false;
    _visualizer = boost::make_shared<pcl::visualization::PCLVisualizer>(_name);
    _visualizer->setBackgroundColor(1.0, 1.0, 1.0);

    while (true) {
        {
            unique_lock<mutex> lock(_mutexViewer);
            if (_clouds.empty())
                continue;

            for (size_t i = 0; i < _clouds.size(); ++i) {
                if (!_inserted[i]) {
                    _visualizer->addPointCloud(_clouds[i], *_colors[i], _cloudNames[i]);
                    _inserted[i] = true;
                } else {
                    _visualizer->updatePointCloud(_clouds[i], *_colors[i], _cloudNames[i]);
                }

                _visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointSize, _cloudNames[i]);
            }
        }

        _visualizer->spinOnce();

        if (checkFinish() || _visualizer->wasStopped())
            break;

        usleep(6000);
    }

    setFinish();
}

Viewer& Viewer::setPointSize(int ps)
{
    unique_lock<mutex> lock(_mutexViewer);
    _pointSize = ps;
    return *this;
}

void Viewer::requestFinish()
{
    unique_lock<mutex> lock(_mutexFinish);
    _finishRequested = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(_mutexFinish);
    return _finished;
}

void Viewer::join()
{
    if (_thread.joinable()) {
        _thread.join();
        cout << "PCDviewer: " << _name << " JOINED!" << endl;
        _visualizer->close();
    }
}

bool Viewer::checkFinish()
{
    unique_lock<mutex> lock(_mutexFinish);
    return _finishRequested;
}

void Viewer::setFinish()
{
    unique_lock<mutex> lock(_mutexFinish);
    _finished = true;
}
