//
// Created by Pablo Aguilar on 8/11/22.
//

#pragma once

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <iostream>
#include <fstream>
#include "common.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

struct Config {
    std::string scans_dir, labels_dir, ranges_dir;
    std::string config_json_filename;
    float voxel_leaf_size;
    float max_range;
    std::vector<label_t> vis_labels;
    uint32_t multithread = 0;
    std::string output_dir;
    uint32_t batch_size = 0;
    uint32_t overlap_size = 0;

    enum class VisualizationMode {
        Semantics,
        Intensity,
        Range
    } vis_mode;

    class LabelConfig {
    public:
        enum class NoiseFilter {
            SOR,
            ROR
        };

        enum class BBoxMethod {
            Rigid,
            Oriented
        };
    private:
        std::map<LabelConfig::NoiseFilter, std::string> noise_filter_names_ = {
                {NoiseFilter::SOR, "SOR"},
                {NoiseFilter::ROR, "ROR"}
        };
        std::map<LabelConfig::BBoxMethod, std::string> bbox_method_names_ = {
                {BBoxMethod::Rigid, "rigid"},
                {BBoxMethod::Oriented, "oriented"}
        };

    public:
        label_t label_id = 0;
        std::string name;
        ColorRGB color;
        bool visualize = true;
        bool use_noise_filter = true;
        bool use_clustering = true;
        NoiseFilter noise_filter = NoiseFilter::ROR;
        struct SORParams {
            uint32_t mean_k = 50;
            float stddev_mul_thres = 0.5f;
        } sor_params;
        struct RORParams {
            float radius = 0.1f;
            uint32_t min_neighbors = 5;
        } ror_params;
        struct EuclideanClusteringParams {
            float cluster_tolerance = 0.25f;
            uint32_t min_cluster_size = 50;
            uint32_t max_cluster_size = 0;
            float min_cluster_volume = 0.f;
            float max_cluster_volume = 0.f;
            BBoxMethod bbox_method = BBoxMethod::Rigid;
        } euclidean_clustering_params;

        void print() const {
            auto width = 30;
            std::cout << "================= LABEL =================\n";
            std::cout << std::left << std::setw(width) << "id:" << label_id << "\n";
            std::cout << std::left << std::setw(width) << "name:" << name << "\n";
            std::cout << std::left << std::setw(width) << "color:" << "[" << color.r<int>() << ", " << color.g<int>() << ", " << color.b<int>() << "]" << "\n";
            std::cout << std::left << std::setw(width) << "visualize:" << std::boolalpha << visualize << "\n";
            std::cout << std::left << std::setw(width) << "noise filter:" << "\n";
            std::cout << std::left << std::setw(width) << "    use:" << std::boolalpha << use_noise_filter << "\n";
            std::cout << std::left << std::setw(width) << "    type:" << noise_filter_names_.find(noise_filter)->second << "\n";
            switch (noise_filter) {
                case NoiseFilter::SOR:
                    std::cout << std::left << std::setw(width) << "    SOR mean k:" << sor_params.mean_k << "\n";
                    std::cout << std::left << std::setw(width) << "    SOR stddev_mul_thres:" << sor_params.stddev_mul_thres << "\n";
                    break;
                case NoiseFilter::ROR:
                    std::cout << std::left << std::setw(width) << "    ROR radius:" << ror_params.radius << "\n";
                    std::cout << std::left << std::setw(width) << "    ROR stddev_mul_thres:" << ror_params.min_neighbors << "\n";
                    break;
            }
            std::cout << std::left << std::setw(width) << "euclidean clustering:" << "\n";
            std::cout << std::left << std::setw(width) << "    use:" << std::boolalpha << use_clustering << "\n";
            std::cout << std::left << std::setw(width) << "    tolerance:" << euclidean_clustering_params.cluster_tolerance << "\n";
            std::cout << std::left << std::setw(width) << "    min cluster size:" << euclidean_clustering_params.min_cluster_size << "\n";
            std::cout << std::left << std::setw(width) << "    max cluster size:" << euclidean_clustering_params.max_cluster_size << "\n";
            std::cout << std::left << std::setw(width) << "    min cluster volume:" << euclidean_clustering_params.min_cluster_volume << "\n";
            std::cout << std::left << std::setw(width) << "    max cluster volume:" << euclidean_clustering_params.max_cluster_volume << "\n";
            std::cout << std::left << std::setw(width) << "    bbox method:" << bbox_method_names_.find(euclidean_clustering_params.bbox_method)->second << "\n";
            std::cout << "=========================================\n\n";
        }
    };

    std::vector<LabelConfig> labels_config;

    void print() const {
        std::cout << "=========== Configuration ===========\n";
        std::cout << "scans_dir:                " << scans_dir << "\n";
        std::cout << "labels_dir:               " << labels_dir << "\n";
        std::cout << "ranges_dir:               " << ranges_dir << "\n";
        std::cout << "config_json_filename:     " << config_json_filename << "\n";
        std::cout << "voxel_leaf_size:          " << voxel_leaf_size << "\n";
        switch (vis_mode) {
            case VisualizationMode::Semantics:
                std::cout << "vis_mode:                 " << "semantics" << "\n";
                break;
            case VisualizationMode::Intensity:
                std::cout << "vis_mode:                 " << "intensity" << "\n";
                break;
            case VisualizationMode::Range:
                std::cout << "vis_mode:                 " << "range" << "\n";
                break;
        }
        std::cout << "max_range:                " << max_range << "\n";
        std::cout << "multithread:              " << multithread << "\n\n";
    }

    bool parseArguments(int argc, char **argv) {
        // Configure options here
        std::string l_vis_mode;
        po::options_description desc("Program options");
        desc.add_options()
                ("help,h", "Print usage message")
                ("scans-dir,i", po::value(&scans_dir), "Folder where scans are stored")
                ("labels-dir,l", po::value(&labels_dir), "Folder where labels are stored")
                ("ranges-dir,r", po::value(&ranges_dir), "Folder where ranges are stored")
                ("config-json", po::value(&config_json_filename)->required()->default_value("./../config_file.json"), "Configuration JSON filename")
                ("output-dir", po::value(&output_dir)->default_value(""), "Output directory")
                ("voxel-leaf-size", po::value(&voxel_leaf_size)->default_value(0.1f), "Voxel leaf size (3 axis)")
                ("vis-mode,v", po::value(&l_vis_mode)->default_value("semantics"), "Visualization mode")
                ("max-range", po::value(&max_range)->default_value(std::numeric_limits<float>::max()), "Max range to load scans")
                ("multithread", po::value(&multithread)->default_value(0), "Number of batches to process concurrently:\n"
                                                                           "  [0]: use maximum\n"
                                                                           "  [1]: single threaded\n"
                                                                           "  [other]: use provided number of threads (limit to maximum threads available)")
                ("batch-size", po::value(&batch_size)->default_value(100), "Number of scans in batch")
                ("overlap-size", po::value(&overlap_size)->default_value(10), "Number of overlapping scans between two consecutive batches")
                ;
        if (argc == 1) {
            std::cerr << desc << "\n";
            return false;
        }

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);
        // Check if there are enough args or if --help is given
        if (vm.count("help")) {
            std::cerr << desc << "\n";
            return false;
        }

        loadLabelsConfig();

        if (l_vis_mode == "semantics")
            vis_mode = VisualizationMode::Semantics;
        else if (l_vis_mode == "intensity")
            vis_mode = VisualizationMode::Intensity;
        else if (l_vis_mode == "range")
            vis_mode = VisualizationMode::Range;

        return true;
    }

    void loadLabelsConfig() {
        std::ifstream f(config_json_filename);
        json data = json::parse(f);

        std::cout << "Labels:\n";
        for (const auto &val: data["labels"].items()) {
            LabelConfig label_config;

            label_config.label_id = strtol(val.key().c_str(), nullptr, 0);
            int r, g, b;
            std::sscanf(to_string(val.value()["color"]).c_str(), "[%i,%i,%i]", &r, &g, &b);
            auto color = ColorRGB(r, g, b);

            label_config.color = color;
            label_config.name = to_string(val.value()["name"]);
            label_config.visualize = val.value()["visualize"];

            label_config.use_noise_filter = val.value()["noise_filter"]["use"];
            if (val.value()["noise_filter"]["type"] == "sor")
                label_config.noise_filter = LabelConfig::NoiseFilter::SOR;
            else if (val.value()["noise_filter"]["type"] == "ror")
                label_config.noise_filter = LabelConfig::NoiseFilter::ROR;

            label_config.sor_params.mean_k = strtol(to_string(val.value()["noise_filter"]["sor"]["mean_k"]).c_str(), nullptr, 0);
            label_config.sor_params.stddev_mul_thres = strtof(to_string(val.value()["noise_filter"]["sor"]["stddev_mul_thres"]).c_str(), nullptr);
            label_config.ror_params.min_neighbors = strtol(to_string(val.value()["noise_filter"]["ror"]["min_neighbors"]).c_str(), nullptr, 0);
            label_config.ror_params.radius = strtof(to_string(val.value()["noise_filter"]["ror"]["radius"]).c_str(), nullptr);

            label_config.use_clustering = val.value()["euclidean_clustering"]["use"];
            label_config.euclidean_clustering_params.cluster_tolerance =
                    strtof(to_string(val.value()["euclidean_clustering"]["cluster_tolerance"]).c_str(), nullptr);
            label_config.euclidean_clustering_params.min_cluster_size =
                    strtol(to_string(val.value()["euclidean_clustering"]["min_cluster_size"]).c_str(), nullptr, 0);
            label_config.euclidean_clustering_params.max_cluster_size =
                    strtol(to_string(val.value()["euclidean_clustering"]["max_cluster_size"]).c_str(), nullptr, 0);
            label_config.euclidean_clustering_params.min_cluster_volume =
                    strtof(to_string(val.value()["euclidean_clustering"]["min_cluster_volume"]).c_str(), nullptr);
            label_config.euclidean_clustering_params.max_cluster_volume =
                    strtof(to_string(val.value()["euclidean_clustering"]["max_cluster_volume"]).c_str(), nullptr);

            if (label_config.euclidean_clustering_params.max_cluster_size == 0)
                label_config.euclidean_clustering_params.max_cluster_size = std::numeric_limits<uint32_t>::max();
            if (label_config.euclidean_clustering_params.max_cluster_volume == 0.f)
                label_config.euclidean_clustering_params.max_cluster_volume = std::numeric_limits<float>::max();

            if (val.value()["euclidean_clustering"]["bbox_method"] == "rigid")
                label_config.euclidean_clustering_params.bbox_method = LabelConfig::BBoxMethod::Rigid;
            else if (val.value()["euclidean_clustering"]["bbox_method"] == "oriented")
                label_config.euclidean_clustering_params.bbox_method = LabelConfig::BBoxMethod::Oriented;

            labels_config.push_back(label_config);
        }
    }
};