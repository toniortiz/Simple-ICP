#include "Algorithms/Icp.h"
#include "Viewer/Viewer.h"
#include <iostream>
#include <pcl/common/time.h>

using namespace std;

/*
 * 1 - source cloud
 * 2 - target cloud
 * 3 - icp iters
 * 4 - max corrs dist
 */
int main()
{
    typedef Icp::PointCloudT PointCloudT;

    Viewer vt("Viewer");
    PointCloudT source = vt.open("/home/antonio/Documents/M.C.C/Tesis/Sample code/Varios/robust_point_cloud_registration-master/samples/bunny/source.pcd");
    PointCloudT target = vt.open("/home/antonio/Documents/M.C.C/Tesis/Sample code/Varios/robust_point_cloud_registration-master/samples/bunny/target.pcd");
    vt.setPointSize(2).start();

    Icp icp;
    auto [status, score, T, aligned] = icp.setIterations(30).setMaxDist(0.3).setEpsilon(1e-9).align(source, target).getResult();

    if (status) {
        vt.add(aligned, "transformed");
    }

    vt.join();

    return 0;
}
