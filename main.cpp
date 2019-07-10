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

    Viewer vc("Viewer");
    vc.setPointSize(2).start();
    PointCloudT source = vc.open("../Data/bunny/source.pcd");
    PointCloudT target = vc.open("../Data/bunny/target.pcd");

    Icp icp;
    auto [status, score, T, aligned] = icp.setIterations(30).setMaxDist(0.3).setEpsilon(1e-9).align(source, target).getResult();

    if (status) {
        vc.add(aligned, "aligned");
        cout << score << endl;
    }

    vc.join();

    return 0;
}
