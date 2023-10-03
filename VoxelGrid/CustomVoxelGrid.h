//
// Created by Pablo Aguilar on 8/11/22.
//

#pragma once

#include <pcl/common/centroid.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"

namespace pcl {
    struct cloud_point_index_idx {
        unsigned int idx;
        unsigned int cloud_point_index;

        cloud_point_index_idx() = default;

        cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(
                cloud_point_index_) {}

        bool operator<(const cloud_point_index_idx &p) const { return (idx < p.idx); }
    };

    template<class PointT>
    class CustomVoxelGrid : public pcl::VoxelGrid<PointT> {
    private:
        using VoxelGrid<PointT>::input_;
        using VoxelGrid<PointT>::filter_field_name_;
        using VoxelGrid<PointT>::indices_;
        using VoxelGrid<PointT>::filter_limit_min_;
        using VoxelGrid<PointT>::inverse_leaf_size_;
        using VoxelGrid<PointT>::min_b_;
        using VoxelGrid<PointT>::max_b_;
        using VoxelGrid<PointT>::div_b_;
        using VoxelGrid<PointT>::divb_mul_;
        using VoxelGrid<PointT>::getClassName;
        using VoxelGrid<PointT>::filter_limit_max_;
        using VoxelGrid<PointT>::filter_limit_negative_;
        using VoxelGrid<PointT>::min_points_per_voxel_;
        using VoxelGrid<PointT>::save_leaf_layout_;
        using VoxelGrid<PointT>::leaf_layout_;
        using VoxelGrid<PointT>::downsample_all_data_;

        void applyFilter(PointCloud<PointT> &output) {
            std::cout << "Filtering input point cloud with size: " << input_->size() << std::endl;

            Eigen::Vector4f min_p, max_p;
            // Get the minimum and maximum dimensions
            if (!filter_field_name_.empty()) // If we don't want to process_batch the entire cloud...
                getMinMax3D<PointT>(input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_),
                                    static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
            else
                getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

            // Check that the leaf size is not too small, given the size of the data
            std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
            std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
            std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

            if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
                PCL_WARN(
                        "[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.\n",
                        getClassName().c_str());
                output = *input_;
                return;
            }

            // Compute the minimum and maximum bounding box values
            min_b_[0] = static_cast<int> (std::floor(min_p[0] * inverse_leaf_size_[0]));
            max_b_[0] = static_cast<int> (std::floor(max_p[0] * inverse_leaf_size_[0]));
            min_b_[1] = static_cast<int> (std::floor(min_p[1] * inverse_leaf_size_[1]));
            max_b_[1] = static_cast<int> (std::floor(max_p[1] * inverse_leaf_size_[1]));
            min_b_[2] = static_cast<int> (std::floor(min_p[2] * inverse_leaf_size_[2]));
            max_b_[2] = static_cast<int> (std::floor(max_p[2] * inverse_leaf_size_[2]));

            // Compute the number of divisions needed along all axis
            div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
            div_b_[3] = 0;

            // Set up the division multiplier
            divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

            // Storage for mapping leaf and pointcloud indexes
            std::vector<cloud_point_index_idx> index_vector;
            index_vector.reserve(indices_->size());

            // If we don't want to process_batch the entire cloud, but rather filter points far away from the viewpoint first...
            if (!filter_field_name_.empty()) {
                // Get the distance field index
                std::vector<pcl::PCLPointField> fields;
                int distance_idx = pcl::getFieldIndex<PointT>(filter_field_name_, fields);
                if (distance_idx == -1) {
                    PCL_ERROR ("[pcl::%s::applyFilter] Invalid filter field name (%s).\n", getClassName().c_str(),
                               filter_field_name_.c_str());
                    return;
                }
                const auto field_offset = fields[distance_idx].offset;

                // First pass: go over all points and insert them into the index_vector vector
                // with calculated idx. Points with the same idx value will contribute to the
                // same point of resulting CloudPoint
                for (const auto &index: (*indices_)) {
                    if (!input_->is_dense)
                        // Check if the point is invalid
                        if (!isXYZFinite((*input_)[index]))
                            continue;

                    // Get the distance value
                    const auto *pt_data = reinterpret_cast<const std::uint8_t *> (&(*input_)[index]);
                    float distance_value = 0;
                    memcpy(&distance_value, pt_data + field_offset, sizeof(float));

                    if (filter_limit_negative_) {
                        // Use a threshold for cutting out points which inside the interval
                        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                            continue;
                    } else {
                        // Use a threshold for cutting out points which are too close/far away
                        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                            continue;
                    }

                    int ijk0 = static_cast<int> (std::floor((*input_)[index].x * inverse_leaf_size_[0]) -
                                                 static_cast<float> (min_b_[0]));
                    int ijk1 = static_cast<int> (std::floor((*input_)[index].y * inverse_leaf_size_[1]) -
                                                 static_cast<float> (min_b_[1]));
                    int ijk2 = static_cast<int> (std::floor((*input_)[index].z * inverse_leaf_size_[2]) -
                                                 static_cast<float> (min_b_[2]));

                    // Compute the centroid leaf index
                    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                    index_vector.emplace_back(static_cast<unsigned int> (idx), index);
                }
            }
                // No distance filtering, process_batch all data
            else {
                // First pass: go over all points and insert them into the index_vector vector
                // with calculated idx. Points with the same idx value will contribute to the
                // same point of resulting CloudPoint
                for (const auto &index: (*indices_)) {
                    if (!input_->is_dense)
                        // Check if the point is invalid
                        if (!isXYZFinite((*input_)[index]))
                            continue;

                    int ijk0 = static_cast<int> (std::floor((*input_)[index].x * inverse_leaf_size_[0]) -
                                                 static_cast<float> (min_b_[0]));
                    int ijk1 = static_cast<int> (std::floor((*input_)[index].y * inverse_leaf_size_[1]) -
                                                 static_cast<float> (min_b_[1]));
                    int ijk2 = static_cast<int> (std::floor((*input_)[index].z * inverse_leaf_size_[2]) -
                                                 static_cast<float> (min_b_[2]));

                    // Compute the centroid leaf index
                    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                    index_vector.emplace_back(static_cast<unsigned int> (idx), index);
                }
            }

            // Second pass: sort the index_vector vector using value representing target cell as index
            // in effect all points belonging to the same output cell will be next to each other
            auto rightshift_func = [](const cloud_point_index_idx &x, const unsigned offset) {
                return x.idx >> offset;
            };
            boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);

            // Third pass: count output cells
            // we need to skip all the same, adjacent idx values
            unsigned int total = 0;
            unsigned int index = 0;
            // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
            // index_vector belonging to the voxel which corresponds to the i-th output point,
            // and of the first point not belonging to.
            std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
            // Worst case size
            first_and_last_indices_vector.reserve(index_vector.size());
            while (index < index_vector.size()) {
                unsigned int i = index + 1;
                while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                    ++i;
                if (i - index >= min_points_per_voxel_) {
                    ++total;
                    first_and_last_indices_vector.emplace_back(index, i);
                }
                index = i;
            }

            // Fourth pass: compute centroids, insert them into their final position
            output.resize(total);
            if (save_leaf_layout_) {
                try {
                    // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
                    std::uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];
                    //This is the number of elements that need to be re-initialized to -1
                    std::uint32_t reinit_size = std::min(static_cast<unsigned int> (new_layout_size),
                                                         static_cast<unsigned int> (leaf_layout_.size()));
                    for (std::uint32_t i = 0; i < reinit_size; i++) {
                        leaf_layout_[i] = -1;
                    }
                    leaf_layout_.resize(new_layout_size, -1);
                }
                catch (std::bad_alloc &) {
                    throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                       "voxel_grid.hpp", "applyFilter");
                }
                catch (std::length_error &) {
                    throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                       "voxel_grid.hpp", "applyFilter");
                }
            }

            index = 0;
            for (const auto &cp: first_and_last_indices_vector) {
                // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
                unsigned int first_index = cp.first;
                unsigned int last_index = cp.second;

                // index is centroid final position in resulting PointCloud
                if (save_leaf_layout_)
                    leaf_layout_[index_vector[first_index].idx] = index;

                //Limit downsampling to coords
                if (!downsample_all_data_) {
                    Eigen::Vector4f centroid(Eigen::Vector4f::Zero ());
                    for (unsigned int li = first_index; li < last_index; ++li)
                        centroid += (*input_)[index_vector[li].cloud_point_index].getVector4fMap();

                    centroid /= static_cast<float> (last_index - first_index);
                    output[index].getVector4fMap() = centroid;
                } else {
                    CentroidPoint<PointT> centroid;

                    struct PointFields {
                        uint32_t label;
                        float range;
                    };
                    std::vector<PointFields> leaf_points;

                    // fill in the accumulator with leaf points
                    for (unsigned int li = first_index; li < last_index; ++li) {
                        centroid.add((*input_)[index_vector[li].cloud_point_index]);
                        leaf_points.push_back({
                                (*input_)[index_vector[li].cloud_point_index].label,
                                (*input_)[index_vector[li].cloud_point_index].range
                        });
                    }

                    // Process centroid (set label and color)
                    centroid.get(output[index]);

                    // Assign range no matter what
                    output[index].range = leaf_points.front().range;

                    if (leaf_points.size() == 1) {
                        output[index].label = leaf_points.front().label;
                    } else if (leaf_points.size() > 1) {
                        auto leaf_points_copy = leaf_points;
                        std::sort(leaf_points_copy.begin(), leaf_points_copy.end(),
                                  [](const PointFields &p1, const PointFields &p2) {
                                      return p1.label < p2.label;
                                  });
                        auto last = std::unique(leaf_points_copy.begin(), leaf_points_copy.end(),
                                                [](const PointFields &p1, const PointFields &p2) {
                                                    return p1.label == p2.label;
                                                });
                        leaf_points_copy.erase(last, leaf_points_copy.end());

                        // leaf_points_copy is the keys (different labels)
                        std::map<uint32_t, uint32_t> labels_map, labels_map_inv;
                        for (const auto &key: leaf_points_copy) {
                            auto label_count = std::count_if(leaf_points.begin(), leaf_points.end(),
                                                             [key](const PointFields &p) {
                                                                 return key.label == p.label;
                                                             });
                            labels_map.insert({label_count, key.label});
                            labels_map_inv.insert({key.label, label_count});
                        }

                        if ((*input_)[0].range == -1) {
                            // if the leaf has different labels, assign the most numerous
                            output[index].label = std::prev(labels_map.end())->second;
                        } else {
                            using ResultsPair = std::pair<uint32_t, float>;
                            using ResultsVector = std::vector<ResultsPair>;
                            ResultsVector results;
                            for (const auto &key: leaf_points_copy) {
                                float sum_range = 0.f;
                                uint16_t count = 0;
                                for (const auto &p: leaf_points) {
                                    if (key.label == p.label) {
                                        sum_range += p.range;
                                        count++;
                                    }
                                }
                                float avg = sum_range / (float) count;
                                results.emplace_back(key.label, (float) labels_map_inv[key.label] / (1 + avg));
                            }
                            output[index].label = std::max_element(results.begin(), results.end(),
                                                                   [](const ResultsPair &p1, const ResultsPair &p2) {
                                                                       return p1.second < p2.second;
                                                                   })->first;
                        }
                    }
                }
                index++;
            }
            output.width = output.size();
        }
    };
}
