//
// Created by Pablo Aguilar on 9/11/22.
//

#pragma once

#include "../common.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

template<class PointT>
class ScanProcess {
protected:
    typename pcl::PointCloud<PointT>::Ptr cloud_ = nullptr;

    typename pcl::PointCloud<PointT>::Ptr filtered_point_cloud_;
public:
    [[nodiscard]] const typename pcl::PointCloud<PointT>::Ptr &getFilteredPointCloud() const {
        return filtered_point_cloud_;
    }

    ScanProcess() : filtered_point_cloud_(new pcl::PointCloud<PointT>()) {};

    void setCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud) {
        cloud_ = cloud;
    }

    virtual void process() = 0;
};

template<class PointT>
class NoiseSORFilter : public ScanProcess<PointT> {
    using ScanProcess<PointT>::cloud_;
    using ScanProcess<PointT>::filtered_point_cloud_;

    int mean_k_ = 50;
    float std_dev_mul_thres_ = 1.0;
public:
    void process() override {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_thres_);
        sor.filter(*filtered_point_cloud_);
    }

    void setMeanK(int meanK) {
        mean_k_ = meanK;
    }

    void setStdDevMulThres(float stdDevMulThres) {
        std_dev_mul_thres_ = stdDevMulThres;
    }

};

template<class PointT>
class NoiseRORFilter : public ScanProcess<PointT> {
    using ScanProcess<PointT>::cloud_;
    using ScanProcess<PointT>::filtered_point_cloud_;

    float radius_search_ = 0.1;
    int min_neighbors_in_radius_ = 5;
public:
    void process() override {
        pcl::RadiusOutlierRemoval<PointT> ror; // Initializing with true will allow us to extract the removed indices
        ror.setInputCloud(cloud_);
        ror.setRadiusSearch(0.1);
        ror.setMinNeighborsInRadius(5);
        ror.filter(*filtered_point_cloud_);
    }

    void setRadiusSearch(float radiusSearch) {
        radius_search_ = radiusSearch;
    }

    void setMinNeighborsInRadius(int minNeighborsInRadius) {
        min_neighbors_in_radius_ = minNeighborsInRadius;
    }
};

class LabelsFilter : public ScanProcess<PointType> {
private:
    std::vector<int32_t> labels_;
public:
    void setLabels(const std::vector<int32_t> &labels);

    void process() override;
};

template<class PointT>
class EuclideanClustering : public ScanProcess<PointT> {
private:
    using ScanProcess<PointT>::cloud_;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_;
    float cluster_tolerance_ = 0.02f;
    int min_cluster_size_ = 100;
    int max_cluster_size_ = 25000;

public:
    void setClusterTolerance(float clusterTolerance) {
        cluster_tolerance_ = clusterTolerance;
    }

    void setMinClusterSize(int minClusterSize) {
        min_cluster_size_ = minClusterSize;
    }

    void setMaxClusterSize(int maxClusterSize) {
        max_cluster_size_ = maxClusterSize;
    }

    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &getClusters() const {
        return clusters_;
    }

    void process() override {
        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance_); // 2cm
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_);
        ec.extract(cluster_indices);

        clusters_.reserve(cluster_indices.size());
        for (const auto &cluster: cluster_indices) {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            cloud_cluster->reserve(cluster.indices.size());
            for (const auto &idx: cluster.indices) {
                cloud_cluster->push_back((*cloud_)[idx]);
            }

            // TODO check cluster volume (set constraints)

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters_.emplace_back(cloud_cluster);
        }
    }
};

template<class PointT>
struct BBData {
    PointT minPoint;
    PointT maxPoint;
    PointT center;
};

template<class PointT>
struct OrientedBBData : public BBData<PointT> {
    Eigen::Vector3f transform;
    Eigen::Quaternionf quaternion;
};

template<class PointT>
OrientedBBData<PointT>
calculateOrientedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cloud, bool restrict_z = false) {
    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);

    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection(new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *cloudPCAprojection);

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = pca.getEigenVectors().transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    Eigen::Quaternionf bboxQuaternion(
            pca.getEigenVectors()); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = pca.getEigenVectors() * meanDiagonal + pcaCentroid.head<3>();

    if (restrict_z) {
        bboxQuaternion.coeffs()[0] = 0.f;
        bboxQuaternion.coeffs()[1] = 0.f;
        bboxQuaternion.coeffs()[2] = 1.f;
    }

    OrientedBBData<PointT> ret;
    ret.quaternion = bboxQuaternion;
    ret.transform = bboxTransform;
    ret.minPoint = minPoint;
    ret.maxPoint = maxPoint;
    return ret;
}

template<class PointT>
BBData<PointT>
calculateRigidBoundingBox(typename pcl::PointCloud<PointT>::Ptr cloud) {
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloud, minPoint, maxPoint);

    BBData<PointT> ret;
    ret.minPoint = minPoint;
    ret.maxPoint = maxPoint;
    PointT center;
    center.x = (maxPoint.x + minPoint.x) / 2.f;
    center.y = (maxPoint.y + minPoint.y) / 2.f;
    center.z = (maxPoint.z + minPoint.z) / 2.f;
    ret.center = center;

    return ret;
}