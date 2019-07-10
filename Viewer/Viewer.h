#ifndef PCDVIEWER_H
#define PCDVIEWER_H

#include "Algorithms/AbstractIcp.h"
#include <mutex>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

class Viewer {
public:
    typedef AbstractIcp::PointT PointT;
    typedef AbstractIcp::PointCloudT PointCloudT;

public:
    Viewer(const std::string& name);

    // Open a PointCloud from file and returns a copy to it
    PointCloudT open(const std::string& filename, const bool& render = true);

    void add(const PointCloudT& cloud, const std::string& name);

    // Call this function to start visualization thread
    Viewer& start();

    Viewer& setPointSize(int ps);

    void requestFinish();

    bool isFinished();

    void join();

protected:
    void run();

    bool checkFinish();
    void setFinish();
    bool _finishRequested;
    bool _finished;
    std::mutex _mutexFinish;

    std::string _name;
    int _pointSize;
    pcl::visualization::PCLVisualizer::Ptr _visualizer;

    std::mutex _mutexViewer;
    std::vector<PointCloudT::Ptr> _clouds;
    std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr> _colors;
    std::vector<std::string> _cloudNames;
    std::vector<bool> _inserted;

    std::thread _thread;
};

#endif // PCDVIEWER_H
