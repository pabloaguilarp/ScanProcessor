#include <iostream>
#include "User/User.h"

// --scans-dir "/Volumes/TOSHIBA EXT/log_2911/world_frame/scan_0000003250-scan_0000003499" --labels-dir "/Volumes/My Passport/results_2911/scan_0000003250-scan_0000003499/predictions" --ranges-dir "/Volumes/My Passport/results_2911/scan_0000003250-scan_0000003499/ranges" --voxel-leaf-size 0.03 -v semantics --max-range 20.0 --multithread 1
// --scans-dir "/Volumes/TOSHIBA EXT/log_2911/world_frame/scan_0000004750-scan_0000004999" --labels-dir "/Volumes/My Passport/results_2911/scan_0000004750-scan_0000004999/predictions" --ranges-dir "/Volumes/My Passport/results_2911/scan_0000004750-scan_0000004999/ranges" --voxel-leaf-size 0.03 -v semantics --max-range 20.0 --multithread 0

int main(int argc, char **argv) {
    Config config;
    if (!config.parseArguments(argc, argv)) {
        throw std::runtime_error("Error parsing arguments");
    }
    config.print();

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->initCameraParameters();

    User user(config);
    user.setViewer(viewer);
    user.setBatches(50, 0);
    user.process();

    viewer->resetCamera();
    viewer->spin();

    return 0;
}
