#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>

#include "argparse.h"
#include "ScanIO/ScanIO.h"
#include "User/User.h"

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
