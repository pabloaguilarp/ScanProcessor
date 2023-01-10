//
// Created by Pablo Aguilar on 9/11/22.
//

#include "ScanProcess.h"

void LabelsFilter::setLabels(const std::vector<int32_t> &labels) {
    labels_ = labels;
}

void LabelsFilter::process() {
    filtered_point_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    size_t filtered_point_cloud_size = std::count_if(cloud_->begin(), cloud_->end(), [this](const PointType &p) {
        return std::any_of(labels_.begin(), labels_.end(), [p](uint32_t val) { return p.label == val; });
    });

    filtered_point_cloud_->reserve(filtered_point_cloud_size);
    for (const auto &p: *cloud_) {
        if (std::any_of(labels_.begin(), labels_.end(), [p](uint32_t val) { return p.label == val; }))
            filtered_point_cloud_->emplace_back(p);
    }
}
