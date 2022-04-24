#pragma once

#include <atomic>
#include <tuple>
#include <vector>

#include "Bvh.hpp"
#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "base/String.hpp"
#include "rtlib.hpp"

namespace FW {

// Given a vector n, forms an orthogonal matrix with n as the last column, i.e.,
// a coordinate system aligned such that n is its local z axis.
// You'll have to fill in the implementation for this.
Mat3f formBasis(const Vec3f& n);

Vec2f getTexelCoords(Vec2f uv, const Vec2i size);

AABB calculateAABB(const std::vector<RTTriangle>& triangles,
                   const std::vector<uint32_t>& indices,
                   const size_t start,
                   const size_t end);

AABB calculateCentroidAABB(const std::vector<RTTriangle>& triangles,
                           const std::vector<uint32_t>& indices,
                           const size_t start,
                           const size_t end);

void buildBVH(const std::vector<RTTriangle>& triangles,
              Bvh& bvh,
              BvhNode& node,
              SplitMode splitMode);

// Main class for tracing rays using BVHs.
class RayTracer {
   public:
    RayTracer(void);
    ~RayTracer(void);

    void constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode);

    void saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles);
    void loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles);

    RaycastResult raycast(const Vec3f& orig, const Vec3f& dir, bool useTextures = true) const;

    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String computeMD5(const std::vector<Vec3f>& vertices);

    std::vector<RTTriangle>* m_triangles;

    void resetRayCounter() { m_rayCount = 0; }
    int getRayCount() { return m_rayCount; }

   private:
    mutable std::atomic<int> m_rayCount;
    // YOUR CODE HERE (R1):
    // This is the library implementation of the ray tracer.
    // Remove this once you have integrated your own ray tracer.
    // std::unique_ptr<rtlib::RayTracer> m_rt;

    Bvh m_bvh;  // Replace the above with your own Bvh and whatever other member variables you have
    std::tuple<int, float, float, float> RayTracer::intersectBVH(const BvhNode& node,
                                                                 const Vec3f& orig,
                                                                 const Vec3f& dir,
                                                                 const Vec3f& invDir,
                                                                 float tmin,
                                                                 bool useTextures) const;
};

}  // namespace FW
