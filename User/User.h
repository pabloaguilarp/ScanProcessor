//
// Created by Pablo Aguilar on 10/11/22.
//

#pragma once

#include "../argparse.h"
#include "../ScanIO/ScanIO.h"

#include "pcl/visualization/pcl_visualizer.h"

#include <mutex>

class LabelsFilter {
private:
    std::vector<int32_t> labels_;
public:
    void setLabels(const std::vector<int32_t>& labels);

    void setCloud(const pcl::PointCloud<PointType>::Ptr& cloud);

    pcl::PointCloud<PointType>::Ptr getFilteredPointCloud() const noexcept;

    void process();

private:
    pcl::PointCloud<PointType>::Ptr cloud_ = nullptr;
    pcl::PointCloud<PointType>::Ptr filtered_point_cloud_ = nullptr;
};

class User {
private:
    Config config_;

    uint32_t batch_size_ = 100;
    uint32_t overlap_size_ = 20;

    ScanIO scan_io_;

    pcl::visualization::PCLVisualizer::Ptr viewer_ = nullptr;

    mutable std::mutex mutex_;

public:
    [[nodiscard]] const pcl::visualization::PCLVisualizer::Ptr &getViewer() const {
        return viewer_;
    }

public:
    void setViewer(const pcl::visualization::PCLVisualizer::Ptr &viewer) {
        viewer_ = viewer;
    }

private:
    template<class PointT>
    void visualizePointCloud_(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& ns) {
        scan_io_.colorize(cloud);
        switch (config_.vis_mode) {
            case Config::VisualizationMode::Semantics: {
                viewer_->addPointCloud<PointType>(cloud, ns);
                break;
            }
            case Config::VisualizationMode::Intensity: {
                pcl::visualization::PointCloudColorHandlerGenericField<PointType> color_handler(cloud, "intensity");
                viewer_->addPointCloud<PointType>(cloud, color_handler, ns);
                break;
            }
            case Config::VisualizationMode::Range: {
                pcl::visualization::PointCloudColorHandlerGenericField<PointType> color_handler(cloud, "range");
                viewer_->addPointCloud<PointType>(cloud, color_handler, ns);
                break;
            }
        }
    }
public:
    User();

    explicit User(Config config);

    void setBatches(uint32_t batch_size, uint32_t overlap_size);

    void process();

    uint32_t process_batch(uint32_t batch_index, uint32_t init_index, uint32_t end_index);
};