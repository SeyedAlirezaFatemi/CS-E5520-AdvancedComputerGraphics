#pragma once

#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

#include "BvhNode.hpp"

namespace FW {

class Bvh {
   public:
    Bvh();
    Bvh(SplitMode mode, size_t triangleCount)
        : mode_(mode),
          rootNode_(std::make_unique<BvhNode>(0, triangleCount)),
          indices_(std::vector<uint32_t>(triangleCount)) {
        std::iota(indices_.begin(), indices_.end(), 0);
    };
    Bvh(std::istream& is);

    // move assignment for performance
    Bvh& operator=(Bvh&& other) {
        mode_ = other.mode_;
        std::swap(rootNode_, other.rootNode_);
        std::swap(indices_, other.indices_);
        return *this;
    }

    Bvh& operator=(Bvh& other) {
        mode_ = other.mode_;
        std::swap(rootNode_, other.rootNode_);
        std::swap(indices_, other.indices_);
        return *this;
    }

    BvhNode& root() { return *rootNode_; }
    const BvhNode& root() const { return *rootNode_; }

    void save(std::ostream& os);

    uint32_t getIndex(uint32_t index) const { return indices_[index]; }

    std::vector<uint32_t>& indices() { return indices_; }
    const std::vector<uint32_t>& indices() const { return indices_; }

   private:
    SplitMode mode_;
    std::unique_ptr<BvhNode> rootNode_;

    std::vector<uint32_t> indices_;  // triangle index list that will be sorted
                                     // during BVH construction
};

}  // namespace FW
