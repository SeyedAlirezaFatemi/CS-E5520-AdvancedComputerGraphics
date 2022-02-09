#define _CRT_SECURE_NO_WARNINGS

#include "RayTracer.hpp"

#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "rtIntersect.inl"
#include "rtlib.hpp"

// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);

namespace FW {

Vec2f getTexelCoords(Vec2f uv, const Vec2i size) {
    // YOUR CODE HERE (R3):
    // Get texel indices of texel nearest to the uv vector. Used in texturing.
    // UV coordinates range from negative to positive infinity. First map them
    // to a range between 0 and 1 in order to support tiling textures, then
    // scale the coordinates by image resolution and find the nearest pixel.

    return Vec2f();
}

Mat3f formBasis(const Vec3f& n) {
    // YOUR CODE HERE (R4):
    return Mat3f();
}

String RayTracer::computeMD5(const std::vector<Vec3f>& vertices) {
    unsigned char digest[16];
    MD5Buffer((void*)&vertices[0], sizeof(Vec3f) * vertices.size(),
              (unsigned int*)digest);

    // turn into string
    char ad[33];
    for (int i = 0; i < 16; ++i) ::sprintf(ad + i * 2, "%02x", digest[i]);
    ad[32] = 0;

    return FW::String(ad);
}

// --------------------------------------------------------------------------

RayTracer::RayTracer() {}

RayTracer::~RayTracer() {}

void RayTracer::loadHierarchy(const char* filename,
                              std::vector<RTTriangle>& triangles) {
    std::ifstream ifs(filename, std::ios::binary);
    m_bvh = Bvh(ifs);

    m_triangles = &triangles;
}

void RayTracer::saveHierarchy(const char* filename,
                              const std::vector<RTTriangle>& triangles) {
    (void)triangles;  // Not used.

    std::ofstream ofs(filename, std::ios::binary);
    m_bvh.save(ofs);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles,
                                   SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // This is where you should construct your BVH.
    Bvh bvh{splitMode, triangles.size()};
    auto rootAABB{calculateAABB(triangles, bvh.indices(), 0, triangles.size())};
    bvh.root().bb = rootAABB;
    buildBVH(triangles, bvh, bvh.root());
    m_bvh = bvh;
    m_triangles = &triangles;
    saveHierarchy("test.hierarchy", *m_triangles);
}

AABB calculateAABB(const std::vector<RTTriangle>& triangles,
                   const std::vector<uint32_t>& indices, const size_t start,
                   const size_t end) {
    AABB result{triangles[indices[start]].min(),
                triangles[indices[start]].max()};
    for (size_t i = start; i < end; i++) {
        const auto& triangleIndex = indices[i];
        const auto& triangle = triangles[triangleIndex];
        result.min = FW::min(result.min, triangle.min());
        result.max = FW::max(result.max, triangle.max());
    }
    return result;
}

AABB calculateCentroidAABB(const std::vector<RTTriangle>& triangles,
                           const std::vector<uint32_t>& indices,
                           const size_t start, const size_t end) {
    AABB result{triangles[indices[start]].centroid(),
                triangles[indices[start]].centroid()};
    for (size_t i = start; i < end; i++) {
        const auto& triangleIndex = indices[i];
        const auto& triangle = triangles[triangleIndex];
        result.min = FW::min(result.min, triangle.centroid());
        result.max = FW::max(result.max, triangle.centroid());
    }
    return result;
}

void buildBVH(const std::vector<RTTriangle>& triangles, Bvh& bvh,
              BvhNode& node) {
    // Note that the spatial median split plane should be based on an AABB
    // spanned by the centroids instead of the actual AABB of the node to ensure
    // an actual split.
    // Don't split this node if it has less than 100 triangles
    if (node.endPrim - node.startPrim < 10) return;
    // Calculate centroid AABB of nodes
    const auto bb = calculateCentroidAABB(triangles, bvh.indices(),
                                          node.startPrim, node.endPrim);
    // Calculate longestAxisIndex
    auto diagonal = bb.max - bb.min;
    size_t longestAxisIndex = 0;
    if (diagonal.z > diagonal.get(longestAxisIndex)) {
        longestAxisIndex = 2;
    }
    if (diagonal.y > diagonal.get(longestAxisIndex)) {
        longestAxisIndex = 1;
    }
    auto axisCenter =
        0.5 * (bb.max.get(longestAxisIndex) + bb.min.get(longestAxisIndex));
    // Partition
    auto& indices = bvh.indices();
    auto iter = std::partition(
        indices.begin() + node.startPrim, indices.begin() + node.endPrim,
        [longestAxisIndex, axisCenter, &triangles](const size_t& nodeIndex) {
            return triangles[nodeIndex].centroid().get(longestAxisIndex) <
                   axisCenter;
        });
    auto partitionIndex = iter - indices.begin();
    // Make two nodes
    node.right = std::make_unique<BvhNode>(node.startPrim, partitionIndex);
    node.left = std::make_unique<BvhNode>(partitionIndex, node.endPrim);
    // Calculate the AABB of the nodes
    node.right->bb = calculateAABB(triangles, bvh.indices(),
                                   node.right->startPrim, node.right->endPrim);
    node.left->bb = calculateAABB(triangles, bvh.indices(),
                                  node.left->startPrim, node.left->endPrim);
    // Recursion
    buildBVH(triangles, bvh, *(node.right));
    buildBVH(triangles, bvh, *(node.left));

    return;
}

std::tuple<int, float, float, float> RayTracer::intersectBVH(
    const BvhNode& node, const Vec3f& orig, const Vec3f& dir,
    const Vec3f& invDir, float tmin) const {
    if (node.right == nullptr) {  // Leaf node
        float umin = 0.0f, vmin = 0.0f;
        int imin = -1;
        float t, u, v;
        for (size_t i = node.startPrim; i < node.endPrim; ++i) {
            auto triangleIndex = m_bvh.getIndex(i);
            const RTTriangle& triangle = (*m_triangles)[triangleIndex];
            if (triangle.intersect_woop(orig, dir, t, u, v)) {
                if (t > 0.0f && t < tmin) {
                    imin = triangleIndex;
                    tmin = t;
                    umin = u;
                    vmin = v;
                }
            }
        }
        return std::make_tuple(imin, tmin, umin, vmin);
    };

    auto rightResult = node.right->bb.intersect(orig, dir, invDir, 0.0f);
    float tright = std::get<1>(rightResult);
    bool didHitRight = std::get<0>(rightResult) && tright < tmin;
    auto leftResult = node.left->bb.intersect(orig, dir, invDir, 0.0f);
    float tleft = std::get<1>(leftResult);
    bool didHitLeft = std::get<0>(leftResult) && tleft < tmin;
    if (!didHitLeft && !didHitRight) {
        return std::make_tuple(-1, 0.0, 0.0, 0.0);
    }
    // int triangleIndex = -1;
    // float tfirst = 0.0;
    bool firstRight = didHitRight && (!didHitLeft || tleft > tright);
    const BvhNode& firstNode = firstRight ? *(node.right) : *(node.left);
    const float tBoundOther = firstRight ? tleft : tright;
    // if (firstRight) {
    //     auto firstIntersectioinResult =
    //         intersectBVH(*(node.right), orig, dir, invDir, tmin);
    // } else {
    //     auto firstIntersectioinResult =
    //         intersectBVH(*(node.left), orig, dir, invDir, tmin);
    // }
    auto firstIntersectioinResult =
        intersectBVH(firstNode, orig, dir, invDir, tmin);
    int firstTriangleIndex = std::get<0>(firstIntersectioinResult);
    float tfirst = std::get<1>(firstIntersectioinResult);
    if ((firstRight && !didHitLeft) || (!firstRight && !didHitRight)) {
        // No other node to check
        return firstIntersectioinResult;
    }
    if (firstTriangleIndex != -1) {
        // Update tmin
        if (tfirst < tmin) {
            tmin = tfirst;
            // This is very important for performance.
            if (tmin < tBoundOther) {
                return firstIntersectioinResult;
            }
        }
    }
    const BvhNode& otherNode = firstRight ? *(node.left) : *(node.right);
    auto otherIntersectioinResult =
        intersectBVH(otherNode, orig, dir, invDir, tmin);
    if (firstTriangleIndex == -1) {
        return otherIntersectioinResult;
    }
    int otherTriangleIndex = std::get<0>(otherIntersectioinResult);
    if (otherTriangleIndex != -1) {
        float tother = std::get<1>(otherIntersectioinResult);
        if (tother > tmin) {
            return firstIntersectioinResult;
        }
        return otherIntersectioinResult;
    }
    return firstIntersectioinResult;
}

RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const {
    ++m_rayCount;

    // YOUR CODE HERE (R1):
    // This is where you traverse the tree you built! It's probably easiest
    // to introduce another function above that does the actual traversal, and
    // use this function only to begin the recursion by calling the traversal
    // function with the given ray and your root node. You can also use this
    // function to do one-off things per ray like finding the elementwise
    // reciprocal of the ray direction.

    auto invDir = 1.0f / dir;

    // You can use this existing code for leaf nodes of the BVH (you do want to
    // change the range of the loop to match the elements the leaf covers.)
    float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
    int imin = -1;
    float t_hit = std::numeric_limits<float>::max();
    // Check if hits the scene then start the recursive call

    RaycastResult castresult;

    // Naive loop over all triangles; this will give you the correct results,
    // but is terribly slow when ran for all triangles for each ray. Try it.
    if (false) {
        for (int i = 0; i < m_triangles->size(); ++i) {
            float t, u, v;
            if ((*m_triangles)[i].intersect_woop(orig, dir, t, u, v)) {
                if (t > 0.0f && t < tmin) {
                    imin = i;
                    tmin = t;
                    umin = u;
                    vmin = v;
                }
            }
        }
    } else {
        auto res = intersectBVH(m_bvh.root(), orig, dir, invDir, tmin);
        imin = std::get<0>(res);
        tmin = std::get<1>(res);
        umin = std::get<2>(res);
        vmin = std::get<3>(res);
    }

    if (imin != -1)
        castresult = RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin,
                                   orig + tmin * dir, orig, dir);

    return castresult;
}

}  // namespace FW
