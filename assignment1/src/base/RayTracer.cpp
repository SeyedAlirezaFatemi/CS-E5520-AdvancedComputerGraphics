#define _CRT_SECURE_NO_WARNINGS

#include "RayTracer.hpp"

#include <stdio.h>

#include <algorithm>
#include <cmath>
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
    uv[0] = uv[0] - FW::floor(uv[0]);
    uv[1] = uv[1] - FW::floor(uv[1]);
    return uv * Vec2f(size);
}

inline F32& minabscoord(Vec3f& v) {
    return FW::filtcoord(v, [](float a, float b) { return FW::abs(a) < FW::abs(b); });
}

Mat3f formBasis(const Vec3f& n) {
    // YOUR CODE HERE (R4):
    Vec3f q{n};
    minabscoord(q) = 1;
    Vec3f t{FW::cross(q, n)};
    Vec3f b{FW::cross(t, n)};
    Mat3f result;
    result.setCol(0, t);
    result.setCol(1, b);
    result.setCol(2, n);
    return result;
}

String RayTracer::computeMD5(const std::vector<Vec3f>& vertices) {
    unsigned char digest[16];
    MD5Buffer((void*)&vertices[0], sizeof(Vec3f) * vertices.size(), (unsigned int*)digest);

    // turn into string
    char ad[33];
    for (int i = 0; i < 16; ++i) ::sprintf(ad + i * 2, "%02x", digest[i]);
    ad[32] = 0;

    return FW::String(ad);
}

// --------------------------------------------------------------------------

RayTracer::RayTracer() {}

RayTracer::~RayTracer() {}

void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles) {
    std::ifstream ifs(filename, std::ios::binary);
    m_bvh = Bvh(ifs);

    m_triangles = &triangles;
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
    (void)triangles;  // Not used.

    std::ofstream ofs(filename, std::ios::binary);
    m_bvh.save(ofs);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // This is where you should construct your BVH.
    Bvh bvh{splitMode, triangles.size()};
    auto rootAABB{calculateAABB(triangles, bvh.indices(), 0, triangles.size())};
    bvh.root().bb = rootAABB;
    buildBVH(triangles, bvh, bvh.root(), splitMode);
    m_bvh = bvh;
    m_triangles = &triangles;
}

AABB calculateAABB(const std::vector<RTTriangle>& triangles,
                   const std::vector<uint32_t>& indices,
                   const size_t start,
                   const size_t end) {
    // Be careful how you index the triangles vector.
    AABB result{triangles[indices[start]].min(), triangles[indices[start]].max()};
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
                           const size_t start,
                           const size_t end) {
    // Be careful how you index the triangles vector.
    AABB result{triangles[indices[start]].centroid(), triangles[indices[start]].centroid()};
    for (size_t i = start; i < end; i++) {
        const auto& triangleIndex = indices[i];
        const auto& triangle = triangles[triangleIndex];
        result.min = FW::min(result.min, triangle.centroid());
        result.max = FW::max(result.max, triangle.centroid());
    }
    return result;
}

int binarySearch(const std::vector<float>& a, float target) {
    int l = 0, r = (int)a.size() - 1, mid, ans = -1;
    while (l <= r) {
        mid = l + (r - l) / 2;
        if (a[mid] == target) return mid;
        if (a[mid] < target) {
            l = mid + 1;
            ans = mid + 1;
        } else {
            ans = mid;
            r = mid - 1;
        }
    }
    return ans;
}

void buildBVH(const std::vector<RTTriangle>& triangles,
              Bvh& bvh,
              BvhNode& node,
              SplitMode splitMode) {
    static const int numIntervals = 8;  // For SplitMode_Sah
    // Don't split this node if it has less than 10 triangles.
    if (node.endPrim - node.startPrim < 10) return;

    int partitionIndex;
    auto& indices = bvh.indices();
    bool sahOK = false;
    if (splitMode == SplitMode_Sah) {
        auto diagonal = node.bb.max - node.bb.min;
        /*
        Assume 5 intervals:
        min            max
        '  '  '  '  '  '
        0  1  2  3  4  5   Partition Ends
          0  1  2  3  4    Partition
          0  1  2  3  4    For Left
          4  3  2  1  0    For Right
        leftBoundingBoxes  => [[0], [0, 1], [0, 1, 2], [0, 1, 2, 3]] * 3 (One for each axis)
        rightBoundingBoxes => [[0], [0, 1], [0, 1, 2], [0, 1, 2, 3]] * 3 (One for each axis)
        partitionEnds      => [1, 2, 3, 4, 5] * 3 (One for each axis)
        We have the following options for partitioning:
        [[0, 1, 2, 3], [4]]
        [[0, 1, 2], [3, 4]]
        [[0, 1], [2, 3, 4]]
        [[0], [1, 2, 3, 4]]
        The bounding boxes in the two vectors that we have, help with deciding which option is best.
        */
        std::vector<std::vector<AABB>> leftBoundingBoxes;
        std::vector<std::vector<AABB>> rightBoundingBoxes;
        std::vector<std::vector<int>> leftNumTriangles;
        std::vector<std::vector<int>> rightNumTriangles;
        std::vector<std::vector<float>> partitionEnds{{}, {}, {}};

        for (size_t axis = 0; axis < 3; axis++) {
            // Add a vector of zeros
            leftNumTriangles.emplace_back(numIntervals - 1, 0);
            rightNumTriangles.emplace_back(numIntervals - 1, 0);
            partitionEnds[axis].reserve(numIntervals - 1);
            // Add empty vectors
            leftBoundingBoxes.emplace_back();
            rightBoundingBoxes.emplace_back();
            float intervalLength = diagonal.get(axis) / static_cast<float>(numIntervals);
            for (size_t intervalIndex = 1; intervalIndex < numIntervals; intervalIndex++) {
                partitionEnds[axis].push_back(node.bb.min.get(axis) +
                                              intervalIndex * intervalLength);
            }
            // numIntervals - 1 options for possible bounding boxes
            for (size_t i = 0; i < numIntervals - 1; i++) {
                leftBoundingBoxes[axis].emplace_back(Vec3f(std::numeric_limits<float>::max()),
                                                     Vec3f(-std::numeric_limits<float>::max()));
                rightBoundingBoxes[axis].emplace_back(Vec3f(std::numeric_limits<float>::max()),
                                                      Vec3f(-std::numeric_limits<float>::max()));
            }
        }
        // Distribute the triangles between intervals
        for (size_t i = node.startPrim; i < node.endPrim; i++) {
            const auto& triangle = triangles[bvh.getIndex(i)];
            const auto triangleCentroid = triangle.centroid();
            for (size_t axis = 0; axis < 3; axis++) {
                /*
                min            max
                '  '  '  '  '  '
                0  1  2  3  4  5
                  0  1  2  3  4    Left
                  4  3  2  1  0    Right
                */
                // Use binary search to find out in which partition this triangle is
                // Just need to compare with partitionEnds[axis]
                int insertionIdx =
                    binarySearch(partitionEnds[axis], triangleCentroid.get(axis));  // 0 -> 4
                // 0 1 2 3 -4-
                for (int i = insertionIdx; i < numIntervals - 1; i++) {
                    leftBoundingBoxes[axis][i].unionWith(triangle);
                    leftNumTriangles[axis][i]++;
                }
                // -4- 3 2 1 0
                for (int i = numIntervals - insertionIdx - 1; i < numIntervals - 1; i++) {
                    rightBoundingBoxes[axis][i].unionWith(triangle);
                    rightNumTriangles[axis][i]++;
                }
            }
        }
        // Find the best
        int bestAxis = 0, leftNum, rightNum;
        float partitionPoint;
        float bestScore = std::numeric_limits<float>::max();
        float score;
        // std::vector<float> scores; // For debug purposes
        for (size_t axis = 0; axis < 3; axis++) {
            float leftArea, rightArea;
            for (size_t intervalIndex = 0; intervalIndex < numIntervals - 1; intervalIndex++) {
                leftNum = leftNumTriangles[axis][intervalIndex];
                rightNum = rightNumTriangles[axis][numIntervals - intervalIndex - 1 - 1];
                if (leftNum == 0 || rightNum == 0) {
                    continue;
                }
                leftArea = leftBoundingBoxes[axis][intervalIndex].area();
                rightArea = rightBoundingBoxes[axis][numIntervals - intervalIndex - 1 - 1].area();
                sahOK = true;
                score = leftArea * leftNum + rightArea * rightNum;
                // scores.push_back(score);
                if (score < bestScore) {
                    bestScore = score;
                    bestAxis = axis;
                    partitionPoint = partitionEnds[axis][intervalIndex];
                }
            }
        }
        if (sahOK) {
            auto iter = std::partition(
                indices.begin() + node.startPrim,
                indices.begin() + node.endPrim,
                [bestAxis, partitionPoint, &triangles](const size_t& triangleIndex) {
                    // ! Crucial to have <= instead of <.
                    // That's because of the way we assign the triangles to the partitions. If the
                    // triangle's centroid is on the partition's end, it is assigned to that
                    // partition.
                    return triangles[triangleIndex].centroid().get(bestAxis) <= partitionPoint;
                });
            partitionIndex = iter - indices.begin();

            // Error case - Does not happen. Let's be safe just in case!
            if (partitionIndex - node.startPrim == 0) {
                sahOK = false;
            }
        }
    }
    if (!sahOK || splitMode != SplitMode_Sah) {
        // SplitMode_SpatialMedian
        // Note that the spatial median split plane should be based on an AABB
        // spanned by the centroids instead of the actual AABB of the node to
        // ensure an actual split. Calculate centroid AABB of nodes
        const auto bb =
            calculateCentroidAABB(triangles, bvh.indices(), node.startPrim, node.endPrim);
        // Find the longest axis
        auto diagonal = bb.max - bb.min;
        size_t longestAxisIndex = 0;
        if (diagonal.z > diagonal.get(longestAxisIndex)) {
            longestAxisIndex = 2;
        }
        if (diagonal.y > diagonal.get(longestAxisIndex)) {
            longestAxisIndex = 1;
        }
        auto axisCenter = 0.5 * (bb.max.get(longestAxisIndex) + bb.min.get(longestAxisIndex));
        // Partition
        auto iter = std::partition(
            indices.begin() + node.startPrim,
            indices.begin() + node.endPrim,
            [longestAxisIndex, axisCenter, &triangles](const size_t& triangleIndex) {
                return triangles[triangleIndex].centroid().get(longestAxisIndex) < axisCenter;
            });
        partitionIndex = iter - indices.begin();
    }
    // Construct two new nodes
    node.right = std::make_unique<BvhNode>(node.startPrim, partitionIndex);
    node.left = std::make_unique<BvhNode>(partitionIndex, node.endPrim);
    // Calculate the AABB of the nodes
    node.right->bb =
        calculateAABB(triangles, bvh.indices(), node.right->startPrim, node.right->endPrim);
    node.left->bb =
        calculateAABB(triangles, bvh.indices(), node.left->startPrim, node.left->endPrim);
    // Recursion
    buildBVH(triangles, bvh, *(node.right), splitMode);
    buildBVH(triangles, bvh, *(node.left), splitMode);
}

float getAlpha(const RTTriangle& triangle, float u, float v) {
    const Texture& alphaTex = triangle.m_material->textures[MeshBase::TextureType_Alpha];
    if (alphaTex.exists()) {
        Vec2f uv{(1.0f - (u + v)) * triangle.m_vertices[0].t + u * triangle.m_vertices[1].t +
                 v * triangle.m_vertices[2].t};
        const Image& img = *alphaTex.getImage();
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        return img.getVec4f(texelCoords).get(0);
    }
    return 1.0f;
}

std::tuple<int, float, float, float> RayTracer::intersectBVH(const BvhNode& node,
                                                             const Vec3f& orig,
                                                             const Vec3f& dir,
                                                             const Vec3f& invDir,
                                                             float tmin,
                                                             bool useTextures) const {
    if (node.right == nullptr) {  // Leaf node
        float umin = 0.0f, vmin = 0.0f;
        int imin = -1;
        float t, u, v;
        for (size_t i = node.startPrim; i < node.endPrim; ++i) {
            auto triangleIndex = m_bvh.getIndex(i);
            const RTTriangle& triangle = (*m_triangles)[triangleIndex];
            if (triangle.intersect_woop(orig, dir, t, u, v)) {
                if (t > 0.0f && t < tmin) {
                    if (!useTextures || getAlpha(triangle, u, v) > 0.5) {  // EXTRA: Alpha Texturing
                        imin = triangleIndex;
                        tmin = t;
                        umin = u;
                        vmin = v;
                    }
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

    // Check any hit
    if (!didHitLeft && !didHitRight) {
        return std::make_tuple(-1, 0.0, 0.0, 0.0);
    }

    // Decide which node we should check first
    bool firstRight = didHitRight && (!didHitLeft || tleft > tright);
    const BvhNode& firstNode = firstRight ? *(node.right) : *(node.left);
    // Lower bound on any possible hit in the other node
    const float tBoundOther = firstRight ? tleft : tright;

    auto firstIntersectionResult = intersectBVH(firstNode, orig, dir, invDir, tmin, useTextures);
    int firstTriangleIndex = std::get<0>(firstIntersectionResult);
    float tfirst = std::get<1>(firstIntersectionResult);

    if ((firstRight && !didHitLeft) || (!firstRight && !didHitRight)) {
        // No other node to check
        return firstIntersectionResult;
    }

    if (firstTriangleIndex != -1) {
        // Update tmin
        if (tfirst < tmin) {
            tmin = tfirst;
            // ! This is very important for performance.
            if (tmin < tBoundOther) {
                // No need to check the other node
                return firstIntersectionResult;
            }
        }
    }

    const BvhNode& otherNode = firstRight ? *(node.left) : *(node.right);
    auto otherIntersectionResult = intersectBVH(otherNode, orig, dir, invDir, tmin, useTextures);

    if (firstTriangleIndex == -1) {
        return otherIntersectionResult;
    }

    int otherTriangleIndex = std::get<0>(otherIntersectionResult);
    if (otherTriangleIndex != -1) {
        float tother = std::get<1>(otherIntersectionResult);
        if (tother > tmin) {
            return firstIntersectionResult;
        }
        return otherIntersectionResult;
    }
    return firstIntersectionResult;
}

RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir, bool useTextures) const {
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
    // Note: tmin should be initialized with 1.0f
    float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
    int imin = -1;

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
        auto res = intersectBVH(m_bvh.root(), orig, dir, invDir, tmin, useTextures);
        imin = std::get<0>(res);
        tmin = std::get<1>(res);
        umin = std::get<2>(res);
        vmin = std::get<3>(res);
    }

    if (imin != -1)
        castresult =
            RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin, orig + tmin * dir, orig, dir);

    return castresult;
}

}  // namespace FW
