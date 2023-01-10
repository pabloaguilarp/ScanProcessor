#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err34-c"
//
// Created by Pablo Aguilar on 7/11/22.
//

#include "ScanIO.h"
#include <iostream>
#include <memory>
#include <optional>
#include <filesystem>
#include <utility>

namespace fs = std::filesystem;

ScanIO::ScanIO() {}

ScanIO::~ScanIO() {}

bool ScanIO::load_labels_(const std::string &filename, std::vector<int32_t> &labels) const {
    // Load binary scan
    std::ifstream input(filename, std::ios::binary);

    // Get file size
    input.seekg(0, input.end);
    const int length = input.tellg();
    input.seekg(0, input.beg);

    const auto buffer_size = length * sizeof(char) / sizeof(float);

    // Read files byte by byte
    auto *buffer = new int32_t[length];
    input.read((char *) buffer, buffer_size * sizeof(int32_t));

    // Read labels
    for (int i = 0; i < buffer_size; i++) {
        labels.push_back(buffer[i]);
    }

    input.close();
    if (!input.good()) {
        std::cout << "\nError occurred at reading time!" << std::endl;
        return false;
    }
    return true;
}

bool ScanIO::load_ranges_(const std::string &filename, std::vector<float> &ranges) const {
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

    // Read labels
    for (int i = 0; i < buffer_size; i++) {
        ranges.push_back(buffer[i]);
    }

    input.close();
    if (!input.good()) {
        std::cout << "\nError occurred at reading time!" << std::endl;
        return false;
    }
    return true;
}

LabelColorMap ScanIO::readColorMap(const std::string &filename) {
    std::ifstream f(filename);
    json data = json::parse(f);

    LabelColorMap ret;
    for (const auto &val: data["labels"].items()) {
        int r, g, b;
        sscanf(to_string(val.value()["color"]).c_str(), "[%i,%i,%i]", &r, &g, &b);

        // std::array<float, 3> value = {(float) r, (float) g, (float) b};
        auto key = (int32_t) atoi(val.key().c_str());

        // std::cout << "Key: " << key << ", color: [" << r << ", " << g << ", " << b << "]" << std::endl;
        const auto color = ColorRGB(r, g, b);
        ret.insert({key, color});
    }

    color_map_ = ret;

    return ret;
}

std::optional<CloudData>
ScanIO::readScan(std::optional<std::string> scan_filename, std::optional<std::string> label_filename,
                 std::optional<std::string> range_filename) const {
    if (!scan_filename) {
        std::cerr << "No scan filename provided. Return" << std::endl;
        return std::nullopt;
    }

    CloudData ret;
    ret.scan_name = fs::path(scan_filename.value()).stem().string();
    std::sscanf(ret.scan_name.c_str(), "%*[^_]_%d.bin", &ret.scan_index);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

    if (!load_scan_<PointType>(scan_filename.value(), cloud)) {
        throw std::runtime_error("Error loading scan");
    }

    std::vector<int32_t> labels;
    if (label_filename) {
        if (!load_labels_(label_filename.value(), labels)) {
            throw std::runtime_error("Error loading labels");
        }
        if (cloud->size() != labels.size()) {
            throw std::runtime_error("Labels file and scan do not contain the same number of points!");
        }
    }

    if (label_filename) {
        if (color_map_.empty()) {
            throw std::runtime_error("Labels file provided but color map is empty!");
        }
        // ret.color_map = color_map_;

        for (int i = 0; i < cloud->size(); i++) {
            cloud->points[i].label = labels[i];
        }
    }

    pcl::Indices indices;
    if (range_filename) {
        std::vector<float> ranges;
        load_ranges_(range_filename.value(), ranges);

        /*std::cout << "ranges: [" << *std::min_element(ranges.begin(), ranges.end()) << ", " <<
                  *std::min_element(ranges.begin(), ranges.end()) << "]\n";*/

        for (int i = 0; i < cloud->size(); i++) {
            cloud->points[i].range = ranges[i];

            if (cloud->points[i].range <= max_range_) {
                indices.push_back(i);
            }
        }
    } else {
        for (int i = 0; i < cloud->size(); i++) {
            indices.push_back(i);
        }
    }

    pcl::PointCloud<PointType>::Ptr cloud_indices(new pcl::PointCloud<PointType>(*cloud, indices));
    ret.cloud = cloud_indices;

    return ret;
}

std::vector<CloudData>
ScanIO::readScans(uint32_t init_index, uint32_t end_index) const {
    // Check init_index and end_index are in range
    if (init_index >= scan_paths_.size() - 1) {

        throw std::runtime_error(std::string("Init index exceeds the number of scans loaded (" + std::to_string(scan_paths_.size()) + ")").c_str());
    }
    // If end_index is greater than scan_paths.size(), trim it to scan_paths.size()
    if (end_index >= scan_paths_.size()) {
        end_index = scan_paths_.size() - 1;
    }

    std::vector<CloudData> ret;
    for (uint32_t i = init_index; i <= end_index; i++) {
        // std::cout << "\rLoading scan " << scan_paths_[i].second;
        auto cloud_data = readScan(scan_paths_[i].first, label_paths_[i].first, range_paths_[i].first);
        if (!cloud_data)
            continue;
        ret.emplace_back(cloud_data.value());
    }
    // std::cout << "\n";

    return ret;
}

std::vector<std::pair<std::string, std::string>>
ScanIO::getFilesInFolder_(const std::optional<std::string> &folder_name, const std::string &extension) {
    std::vector<std::pair<std::string, std::string>> ret;
    if (!folder_name)
        return ret;
    if (folder_name == "")
        return ret;

    for (auto &p: fs::directory_iterator(folder_name.value()))
        if (p.path().extension() == extension)
            ret.emplace_back(fs::absolute(p.path()).string(), p.path().stem().string());

    std::sort(ret.begin(), ret.end(), [](
            const std::pair<std::string, std::string> &s1, const std::pair<std::string, std::string> &s2) {
        uint32_t num1, num2;
        std::sscanf(s1.second.c_str(), "%*[^_]_%d.bin", &num1);
        std::sscanf(s2.second.c_str(), "%*[^_]_%d.bin", &num2);
        return num1 < num2;
    });

    return ret;
}

/*ScanIO::ScanIO(std::string color_map_json_filename) : color_map_json_filename_(std::move(color_map_json_filename)) {
    readColorMap(color_map_json_filename_);
    readLabelsMap(color_map_json_filename_);
}*/

void ScanIO::colorize(const pcl::PointCloud<PointType>::Ptr &cloud) {
    for (auto &p: *cloud) {
        p.r = color_map_[p.label].r<int>();
        p.g = color_map_[p.label].g<int>();
        p.b = color_map_[p.label].b<int>();
    }
}

const LabelColorMap &ScanIO::getColorMap() const {
    return color_map_;
}

LabelsMap ScanIO::readLabelsMap(const std::string &filename) {
    std::ifstream f(filename);
    json data = json::parse(f);

    LabelsMap ret;
    for (const auto &val: data["labels"].items()) {
        auto key = (int32_t) atoi(val.key().c_str());
        ret.insert({key, to_string(val.value()["name"]).c_str()});
    }

    labels_map_ = ret;
    return ret;
}

const LabelsMap &ScanIO::getLabelsMap() const {
    return labels_map_;
}

void ScanIO::setMaxRange(float maxRange) {
    max_range_ = maxRange;
}

#pragma clang diagnostic pop