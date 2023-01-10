//
// Created by Pablo Aguilar on 7/11/22.
//

#pragma once

#include "../common.h"
#include <nlohmann/json.hpp>
#include <map>
#include <array>
#include <tuple>

using json = nlohmann::json;

struct CloudData {
    pcl::PointCloud<PointType>::Ptr cloud = nullptr;
    // std::map<int32_t, std::array<float, 3>> color_map = {};
    std::string scan_name;
    uint32_t scan_index;
};

// using ColorArray = std::array<float, 3>;
// using ColorMap = std::map<int32_t, ColorArray>;
using LabelsMap = std::map<int32_t, std::string>;

class ScanIO {
private:
    LabelColorMap color_map_ = {};
    LabelsMap labels_map_ = {};
    std::string color_map_json_filename_;

    static std::vector<std::pair<std::string, std::string>>
    getFilesInFolder_(const std::optional<std::string> &folder_name, const std::string &extension);

    std::vector<std::pair<std::string, std::string>> scan_paths_;
    std::vector<std::pair<std::string, std::string>> label_paths_;
    std::vector<std::pair<std::string, std::string>> range_paths_;

    float max_range_ = std::numeric_limits<float>::max();

public:
    void setMaxRange(float maxRange);

public:
    ScanIO();

    /*
    explicit ScanIO(std::string color_map_json_filename);
    */

    virtual ~ScanIO();

    void loadFiles(const std::optional<std::string> &scans_folder, const std::optional<std::string> &labels_folder,
                   const std::optional<std::string> &ranges_folder) {
        if (!scans_folder) {
            throw std::runtime_error("No scans folder provided");
        }

        scan_paths_ = getFilesInFolder_(scans_folder, ".bin");
        label_paths_ = getFilesInFolder_(labels_folder, ".label");
        range_paths_ = getFilesInFolder_(ranges_folder, ".range");

        if (scan_paths_.size() != label_paths_.size()) {
            throw std::runtime_error("Scans folder and labels folder do not contain the same number of files");
        }
        if (scan_paths_.size() != range_paths_.size()) {
            std::string msg = "Scans folder and ranges folder do not contain the same number of files (" +
                              std::to_string(scan_paths_.size()) + " vs " +
                              std::to_string(range_paths_.size()) + ")";
            throw std::runtime_error(msg);
        }
    }

    [[nodiscard]] size_t getNumFiles() const {
        return scan_paths_.size();
    }

    [[nodiscard]] std::optional<CloudData>
    readScan(std::optional<std::string> scan_filename, std::optional<std::string> label_filename,
             std::optional<std::string> range_filename) const;

    [[nodiscard]] std::vector<CloudData>
    readScans(uint32_t init_index, uint32_t end_index) const;

    LabelColorMap readColorMap(const std::string &filename);

    [[nodiscard]] const LabelColorMap &getColorMap() const;

    [[nodiscard]] const LabelsMap &getLabelsMap() const;

    LabelsMap readLabelsMap(const std::string &filename);

    [[nodiscard]] const std::string &getLabelName(int32_t index) {
        return labels_map_[index];
    }

    [[nodiscard]] int32_t getLabelIndex(const std::string &label_name) {
        return std::find_if(labels_map_.begin(), labels_map_.end(),
                            [label_name](const std::pair<int32_t, std::string> &p) {
                                return label_name == p.second;
                            })->first;
    }

    /*[[nodiscard]] const ColorArray &getLabelColor(int32_t index) {
        return color_map_[index];
    }*/

    void
    colorize(const pcl::PointCloud<PointType>::Ptr &cloud);

private:
    bool load_labels_(const std::string &filename, std::vector<int32_t> &labels) const;

    bool load_ranges_(const std::string &filename, std::vector<float> &ranges) const;

    template<class PointT>
    bool load_scan_(const std::string &filename, typename pcl::PointCloud<PointT>::Ptr &cloud) const {
        // Load binary scan
        std::ifstream input(filename, std::ios::binary);

        // Get file size
        input.seekg(0, input.end);
        const int length = input.tellg();
        input.seekg(0, input.beg);

        const auto buffer_size = length * sizeof(char) / sizeof(float);

        // Read files byte by byte
        auto *buffer = new float[length];
        input.read((char *) buffer, buffer_size * sizeof(float));

        if (cloud == nullptr)
            cloud = std::make_shared<pcl::PointCloud<PointT>>();

        for (int i = 0; i < buffer_size; i += 4) {
            auto p = PointT();
            p.x = buffer[i];
            p.y = buffer[i + 1];
            p.z = buffer[i + 2];
            p.intensity = buffer[i + 3];
            cloud->push_back(p);
        }

        input.close();
        if (!input.good()) {
            std::cout << "\nError occurred at reading time!" << std::endl;
            return false;
        }
        return true;
    }
};
