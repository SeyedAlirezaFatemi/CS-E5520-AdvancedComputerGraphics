#pragma once

/*
 * utilities for internal use etc. in the raytracer class and the bvh
 * construction
 */

#include <iostream>
#include <limits>
#include <tuple>

#include "base/Math.hpp"

namespace FW {

enum SplitMode {
    SplitMode_SpatialMedian,
    SplitMode_ObjectMedian,
    SplitMode_Sah,
    SplitMode_None,
    SplitMode_Linear
};

struct Plane : public Vec4f {
    inline float dot(const Vec3f& p) const { return p.x * x + p.y * y + p.z * z + w; }
};

struct AABB {
    Vec3f min, max;
    inline AABB() : min(), max() {}
    inline AABB(const Vec3f& min, const Vec3f& max) : min(min), max(max) {}
    inline F32 area() const {
        Vec3f d(max - min);
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    };
    void unionWith(const RTTriangle& triangle) {
        min = FW::min(min, triangle.min());
        max = FW::max(max, triangle.max());
    };
    std::tuple<bool, float> intersect(const Vec3f& orig,
                                      const Vec3f& dir,
                                      const Vec3f& invDir,
                                      const float tmin) const {
        float tstart = -std::numeric_limits<float>::max(), tend = std::numeric_limits<float>::max();
        float t1, t2;
        for (size_t i = 0; i < 3; i++) {
            if (dir.get(i) == 0.0) {
                if (orig.get(i) < min.get(i) || orig.get(i) > max.get(i))
                    return std::make_tuple(false, 0.0f);
            } else {
                t1 = (min.get(i) - orig.get(i)) * invDir.get(i);
                t2 = (max.get(i) - orig.get(i)) * invDir.get(i);
                if (t1 > t2) std::swap(t1, t2);
                tstart = FW::max(tstart, t1);
                tend = FW::min(tend, t2);
            }
        }
        return std::make_tuple(!(tstart > tend || tend < tmin), tstart);
    }
};

inline std::ostream& operator<<(std::ostream& os, const FW::Vec3f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

inline std::ostream& operator<<(std::ostream& os, const FW::Vec4f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
}

inline std::ostream& operator<<(std::ostream& os, const AABB& bb) {
    return os << "BB(" << bb.min << ", " << bb.max << ")";
}

}  // namespace FW
