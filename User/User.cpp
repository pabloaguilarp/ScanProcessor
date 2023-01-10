//
// Created by Pablo Aguilar on 10/11/22.
//

#include "User.h"

#include <memory>
#include <utility>
#include <future>
#include <chrono>
#include "../VoxelGrid/CustomVoxelGrid.h"
#include "../ScanProcess/ScanProcess.h"

#define MUTEX_GUARD(code) mutex_.lock(); \
code                                    \
mutex_.unlock();

User::User() {}

User::User(Config config) : config_(std::move(config)) {
    scan_io_.setMaxRange(config_.max_range);
    scan_io_.readColorMap(config_.config_json_filename);
    scan_io_.readLabelsMap(config_.config_json_filename);
    scan_io_.loadFiles(config_.scans_dir, config_.labels_dir, config_.ranges_dir);
}

uint32_t User::process_batch(uint32_t batch_index, uint32_t init_index = 0, uint32_t end_index = UINT32_MAX) {
    std::vector<CloudData> cloud_data;
    try {
        cloud_data = scan_io_.readScans(init_index, end_index);
        MUTEX_GUARD(
                std::cout << "Number of scans loaded: " << cloud_data.size() << std::endl;
        )
    } catch (std::exception &e) {
        mutex_.lock();
        std::cerr << e.what() << std::endl;
        mutex_.unlock();
        return batch_index;
    }

    MUTEX_GUARD(
            if (viewer_ == nullptr) {
                viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Viewer");
                viewer_->initCameraParameters();
            }
    )

    pcl::PointCloud<PointType>::Ptr cloud_accum(new pcl::PointCloud<PointType>());
    size_t accum_size = 0;
    for (const auto &cloud: cloud_data) {
        accum_size += cloud.cloud->size();
    }
    cloud_accum->reserve(accum_size);

    // Accumulate point cloud
    for (const auto &cloud: cloud_data) {
        for (int i = 0; i < cloud.cloud->size(); i++) {
            cloud_accum->emplace_back(cloud.cloud->at(i));
        }
    }

    MUTEX_GUARD(
            std::cout << "Cloud accum size: " << cloud_accum->size() << std::endl;
    )


    // Voxelize
    pcl::CustomVoxelGrid<PointType> voxel;
    voxel.setInputCloud(cloud_accum);
    voxel.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
    pcl::PointCloud<PointType>::Ptr cloud_voxel(new pcl::PointCloud<PointType>());
    voxel.filter(*cloud_voxel);

    MUTEX_GUARD(
            std::cout << "\nBatch index: " << batch_index << "\n";
            std::cout << "Number of points in accumulated point cloud: " << cloud_accum->size() << std::endl;
            std::cout << "Number of points in voxelized point cloud: " << cloud_voxel->size() << std::endl;
            std::cout << "Downsampling ratio: " << (float) cloud_voxel->size() / (float) cloud_accum->size() * 100.f
                      << "%\n";
    )

    for (auto &val: config_.labels_config) {
        if (!val.visualize) {
            continue;
        }

        // Filter by label
        LabelsFilter labels_filter;
        labels_filter.setCloud(cloud_voxel);
        labels_filter.setLabels({val.label_id});
        labels_filter.process();
        auto label_cloud = labels_filter.getFilteredPointCloud();

        if (label_cloud->empty())
            continue;

        pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>());
        if (val.use_noise_filter) {
            switch (val.noise_filter) {
                case Config::LabelConfig::NoiseFilter::SOR: {
                    NoiseSORFilter<PointType> noise_filter;
                    noise_filter.setCloud(label_cloud);
                    noise_filter.setMeanK(val.sor_params.mean_k);
                    noise_filter.setStdDevMulThres(val.sor_params.stddev_mul_thres);
                    noise_filter.process();
                    filtered_cloud = noise_filter.getFilteredPointCloud();
                    break;
                }
                case Config::LabelConfig::NoiseFilter::ROR: {
                    NoiseRORFilter<PointType> noise_filter;
                    noise_filter.setCloud(label_cloud);
                    noise_filter.setRadiusSearch(val.ror_params.radius);
                    noise_filter.setMinNeighborsInRadius(val.ror_params.min_neighbors);
                    noise_filter.process();
                    filtered_cloud = noise_filter.getFilteredPointCloud();
                    break;
                }
            }
        } else {
            filtered_cloud = label_cloud;
        }

        if (filtered_cloud->empty())
            continue;

        if (!config_.output_dir.empty()) {
            MUTEX_GUARD(
                    std::filesystem::path path(config_.output_dir);
                    auto filename =
                            "filtered_cloud_b" + std::to_string(batch_index) + "_l" + std::to_string(val.label_id) + ".pcd";
                    path.append(filename);
                    try {
                        pcl::io::savePCDFile<PointType>(path.string(), *filtered_cloud, true);
                        std::cout << "Saved file: " << path << "\n";
                    } catch (std::exception &e) {
                        std::cerr << e.what() << "\n";
                    }
            )
        }

        if (val.use_clustering) {
            // Euclidean clustering
            EuclideanClustering<PointType> ec;
            ec.setCloud(filtered_cloud);
            ec.setClusterTolerance(val.euclidean_clustering_params.cluster_tolerance);
            ec.setMinClusterSize(val.euclidean_clustering_params.min_cluster_size);
            ec.setMaxClusterSize(val.euclidean_clustering_params.max_cluster_size);
            ec.process();
            const auto clusters = ec.getClusters();
            MUTEX_GUARD(
                    std::cout << clusters.size() << " clusters detected\n";
            )
            if (clusters.empty())
                continue;

            // Calculate bounding box for each cluster
            int j = 0;
            for (const auto &cluster: clusters) {
                if (cluster->empty()) {
                    j++;
                    continue;
                }

                BBData<PointType> bb_data;
                const auto ns = std::string("bbox_") + std::to_string(j) + "_" + std::to_string(val.label_id) + "_" +
                                std::to_string(batch_index);

                if (val.euclidean_clustering_params.bbox_method == Config::LabelConfig::BBoxMethod::Oriented) {
                    MUTEX_GUARD(
                            std::cerr << "Oriented bounding box method in not implemented currently. "
                                         "Rigid bounding box will be calculated\n";
                            val.euclidean_clustering_params.bbox_method = Config::LabelConfig::BBoxMethod::Rigid;
                    )
                }

                switch (val.euclidean_clustering_params.bbox_method) {
                    case Config::LabelConfig::BBoxMethod::Oriented: {
                        break;
                    }
                    case Config::LabelConfig::BBoxMethod::Rigid: {
                        const auto bbox = calculateRigidBoundingBox<PointType>(cluster);
                        /*viewer_->addCube(bbox.minPoint.x, bbox.maxPoint.x,
                                         bbox.minPoint.y, bbox.maxPoint.y,
                                         bbox.minPoint.z, bbox.maxPoint.z,
                                         1.0, 1.0, 1.0,
                                         ns);*/
                        bb_data = bbox;
                        break;
                    }
                }

                auto bb_length = bb_data.maxPoint.x - bb_data.minPoint.x;
                auto bb_width = bb_data.maxPoint.y - bb_data.minPoint.y;
                auto bb_height = bb_data.maxPoint.z - bb_data.minPoint.z;
                auto bb_volume = bb_length * bb_width * bb_height;

                if (bb_volume < val.euclidean_clustering_params.min_cluster_volume) {
                    j++;
                    continue;
                }
                if (bb_volume > val.euclidean_clustering_params.max_cluster_volume) {
                    j++;
                    continue;
                }

                MUTEX_GUARD(
                        if (val.visualize) {
                            viewer_->addCube(bb_data.minPoint.x, bb_data.maxPoint.x,
                                             bb_data.minPoint.y, bb_data.maxPoint.y,
                                             bb_data.minPoint.z, bb_data.maxPoint.z,
                                             1.0, 1.0, 1.0,
                                             ns);
                            viewer_->setShapeRenderingProperties(
                                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                    ns);
                            viewer_->addText3D(scan_io_.getLabelName(val.label_id),
                                               pcl::PointXYZ(bb_data.center.x, bb_data.center.y,
                                                             bb_data.maxPoint.z + 1.f),
                                               0.5, 1.0, 1.0, 1.0, ns + "_text");

                            visualizePointCloud_<PointType>(cluster,
                                                            "cluster_" + std::to_string(j) + "_label_" +
                                                            std::to_string(val.label_id)
                                                            + "_" + std::to_string(batch_index));
                        }
                )

                j++;
            }
        } else {
            if (val.visualize) {
                MUTEX_GUARD(
                        visualizePointCloud_<PointType>(filtered_cloud,
                                                        "cloud_label_" + std::to_string(val.label_id)
                                                        + "_" + std::to_string(batch_index));
                )
            }
        }
    }

    return batch_index;
}

void User::setBatches(uint32_t batch_size, uint32_t overlap_size) {
    if (overlap_size >= batch_size)
        throw std::runtime_error("Overlap size must be smaller than batch size");

    batch_size_ = batch_size;
    overlap_size_ = overlap_size;
}

void User::process() {
    // Set up batches
    const auto num_files = scan_io_.getNumFiles();
    std::cout << "Loaded " << num_files << " files\n";

    struct BatchIndices {
        uint32_t batch_id;
        uint32_t init_index;
        uint32_t end_index;
    };
    using BatchGroup = std::vector<BatchIndices>;
    BatchGroup batches;
    uint32_t init_index = 0;
    uint32_t batch_counter = 0;
    while (true) {
        uint32_t end_index = init_index + batch_size_;
        batches.push_back({batch_counter, init_index, end_index});
        if (end_index > num_files)
            break;

        init_index = end_index - overlap_size_;
        batch_counter++;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (config_.multithread == 1) {
        std::cout << "Using " << config_.multithread << " threads. No multithreading\n";

        for (const auto &batch: batches) {
            std::cout << "Processing batch (" << batch.init_index << ", " << batch.end_index << ")...\n";
            process_batch(batch.batch_id, batch.init_index, batch.end_index);
        }
    } else {
        if (config_.multithread == 0 ||
            config_.multithread > std::thread::hardware_concurrency()) {
            config_.multithread = std::thread::hardware_concurrency();
        }

        std::vector<BatchGroup> concurrent_batches_group;
        BatchGroup current_concurrent_group;
        for (int i = 0; i < batches.size(); i++) {
            current_concurrent_group.emplace_back(batches[i]);
            if (current_concurrent_group.size() == config_.multithread ||
                i == batches.size() - 1) {
                concurrent_batches_group.push_back(current_concurrent_group);
                current_concurrent_group.clear();
            }
        }

        std::cout << "Using " << config_.multithread << " threads. " << concurrent_batches_group.size()
                  << " batches groups created\n";

        int bgg_counter = 0;
        for (const auto &bgg: concurrent_batches_group) {
            std::cout << "\n============= Processing batch group " << bgg_counter << " =============\n";
            std::vector<std::future<uint32_t>> futures;
            for (const auto &batch: bgg) {
                std::cout << "Processing batch (" << batch.init_index << ", " << batch.end_index << ")...\n";
                futures.emplace_back(
                        std::async(std::launch::async, &User::process_batch, this, batch.batch_id, batch.init_index,
                                   batch.end_index)
                );
            }

            for (auto &fut: futures) {
                auto batch_index = fut.get();
                std::cout << "Finished batch " << batch_index << "\n";
            }
            std::cout << "============= Finished batch group " << bgg_counter << " ===============\n\n";
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

    std::cout << "\nFinished all\n\n";
}
