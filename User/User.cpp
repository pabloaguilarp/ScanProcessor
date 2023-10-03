//
// Created by Pablo Aguilar on 10/11/22.
//

#include "User.h"

#include <memory>
#include <utility>
#include <future>
#include <chrono>

#define MUTEX_GUARD(code) mutex_.lock(); \
code                                    \
mutex_.unlock();

void LabelsFilter::setLabels(const std::vector<int32_t>& labels)
{
	labels_ = labels;
}

void LabelsFilter::setCloud(const pcl::PointCloud<PointType>::Ptr& cloud)
{
	cloud_ = cloud;
}

pcl::PointCloud<PointType>::Ptr LabelsFilter::getFilteredPointCloud() const noexcept
{
	return filtered_point_cloud_;
}

void LabelsFilter::process()
{
	filtered_point_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

	size_t filtered_point_cloud_size = std::count_if(cloud_->begin(), cloud_->end(), [this](const PointType& p) {
		return std::any_of(labels_.begin(), labels_.end(), [p](uint32_t val) { return p.label == val; });
	});

	filtered_point_cloud_->reserve(filtered_point_cloud_size);
	for (const auto& p : *cloud_) {
		if (std::any_of(labels_.begin(), labels_.end(), [p](uint32_t val) { return p.label == val; }))
			filtered_point_cloud_->emplace_back(p);
	}
}

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


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<PointType>::Ptr cloud_voxel(new pcl::PointCloud<PointType>());
    cloud_voxel = cloud_accum;

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
        filtered_cloud = label_cloud;

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
