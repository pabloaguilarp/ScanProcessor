//
// Created by Pablo Aguilar on 7/11/22.
//

#pragma once

#include <string>

#include "pcl/point_cloud.h"


#define PCL_NO_PRECOMPILE

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <cstdint>
#include <type_traits>
#include <map>

namespace pcl {
    struct EIGEN_ALIGN16 PointXYZIL {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t label;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint16_t, label, label)
)

namespace pcl {
    struct EIGEN_ALIGN16 _PointXYZIRGBL {
        PCL_ADD_POINT4D;
        union {
            struct {
                float intensity;
            };
            float data_c[4];
        };
        PCL_ADD_RGB;
        uint32_t label;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct EIGEN_ALIGN16 PointXYZIRGBL : public _PointXYZIRGBL {
        inline constexpr PointXYZIRGBL() :
                PointXYZIRGBL{0, 0, 0, 0, 0, 0, 0} {}

        inline constexpr PointXYZIRGBL(const _PointXYZIRGBL &p) :
        PointXYZIRGBL{p.x, p.y, p.z, p.intensity, p.r, p.g, p.b, p.label} {}

        inline constexpr PointXYZIRGBL(float _x, float _y, float _z, float _intensity, std::uint8_t _r,
                                       std::uint8_t _g, std::uint8_t _b, std::uint32_t _label = 0) :
                                       _PointXYZIRGBL{{{_x, _y, _z, 1.0f}}, _intensity, {{{_b, _g, _r, 255}}}, _label} {}
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBL,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (uint32_t, rgb, rgb)
                                          (float, intensity, intensity)
                                          (uint16_t, label, label)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIRGBL, pcl::_PointXYZIRGBL)

namespace pcl {
    struct EIGEN_ALIGN16 _PointXYZIRRGBL {
        PCL_ADD_POINT4D;
        union {
            struct {
                float intensity;
            };
            float data_c[4];
        };
        union {
            struct {
                float range;
            };
            float data_r[4];
        };
        PCL_ADD_RGB;
        uint32_t label;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct EIGEN_ALIGN16 PointXYZIRRGBL : public _PointXYZIRRGBL {
        inline constexpr PointXYZIRRGBL() :
                PointXYZIRRGBL{0, 0, 0, -1, -1, 0, 0, 0, 0} {}

        inline constexpr PointXYZIRRGBL(const _PointXYZIRRGBL &p) :
                PointXYZIRRGBL{p.x, p.y, p.z, p.intensity, p.range, p.r, p.g, p.b, p.label} {}

        inline constexpr PointXYZIRRGBL(float _x, float _y, float _z, float _intensity, float _range, std::uint8_t _r,
                                       std::uint8_t _g, std::uint8_t _b, std::uint32_t _label = 0) :
                _PointXYZIRRGBL{{{_x, _y, _z, 1.0f}}, _intensity, _range, {{{_b, _g, _r, 255}}}, _label} {}
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRRGBL,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, range, range)
                                          (uint32_t, rgb, rgb)
                                          (uint16_t, label, label)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIRRGBL, pcl::_PointXYZIRRGBL)

class PointXYZIRRGBL {
    float x;
    float y;
    float z;
    float intensity;
    float range;
    uint32_t rgb;
    uint16_t label;
};

using PointType = pcl::PointXYZIRRGBL;

// =================== Common structs and typedefs ===================
using label_t = int32_t;

struct ColorRGB {
    ColorRGB() = default;

    template<typename T = float>
    ColorRGB(T r, T g, T b) {
        if (std::is_same_v<T, float>) {
            r_ = r;
            g_ = g;
            b_ = b;
        } else if (std::is_same_v<T, double>) {
            r_ = static_cast<float>(r);
            g_ = static_cast<float>(g);
            b_ = static_cast<float>(b);
        } else {
            r_ = static_cast<float>(r) / 255.f;
            g_ = static_cast<float>(g) / 255.f;
            b_ = static_cast<float>(b) / 255.f;
        }
    }

    template<typename T = float>
    [[nodiscard]] T r() const { return convert_field_<T>(r_); }
    template<typename T = float>
    [[nodiscard]] T g() const { return convert_field_<T>(g_); }
    template<typename T = float>
    [[nodiscard]] T b() const { return convert_field_<T>(b_); }

    //friend std::ostream& operator<<(std::ostream& os, const ColorRGB& color);

    /*std::ostream& operator<<(std::ostream& os) const
    {
        os << "[" << r<int>() << ", " << g<int>() << ", " << b<int>() << "]";
        return os;
    }*/
private:
    float r_ = 0.f;
    float g_ = 0.f;
    float b_ = 0.f;

    template<typename T = float>
    [[nodiscard]] T convert_field_(float field) const {
        if (std::is_same_v<T, float>) {
            return field;
        } else if (std::is_same_v<T, double>) {
            return static_cast<double>(field);
        } else {
            return static_cast<T>(field * 255.f);
        }
    }
};

using LabelColor = std::pair<label_t, ColorRGB>;
using LabelColorMap = std::map<label_t, ColorRGB>;