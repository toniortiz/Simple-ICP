#include "Algorithms/Gicp.h"
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
    typedef AbstractIcp::PointCloudT PointCloudT;

    Viewer vc("Viewer");
    vc.setPointSize(2).start();
    PointCloudT source = vc.open("../Data/bunny/source.pcd");
    // PointCloudT target = vc.open("../Data/bunny/target.pcd");

    float theta = M_PI / 8;
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    M(0, 0) = cos(theta);
    M(0, 1) = -sin(theta);
    M(1, 0) = sin(theta);
    M(1, 1) = cos(theta);
    M(0, 3) = 0.1f;
    M(1, 3) = 0.1f;
    M(2, 3) = 0.1f; // 0.1 m in z

    cout << "Original motion" << endl;
    cout << M << endl;

    PointCloudT target;
    pcl::transformPointCloud(source, target, M);
    vc.add(target, "target");

    Icp icp;
    auto [status, score, T, aligned] = icp.setIterations(100).setMaxDist(0.3).setEpsilon(1e-9).align(source, target).getResult();

    if (status) {
        vc.add(aligned, "aligned");
        cout << "\nEstimated motion" << endl;
        cout << T << endl;
    }

    vc.join();

    return 0;
}
