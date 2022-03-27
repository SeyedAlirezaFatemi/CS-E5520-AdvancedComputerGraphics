#pragma once


#include "BvhNode.hpp"


#include <vector>
#include <iostream>
#include <memory>


namespace FW {


class Bvh {
public:

    Bvh();
    Bvh(std::istream& is);

    // move assignment for performance
    Bvh& operator=(Bvh&& other) {
        mode_ = other.mode_;
        std::swap(rootNode_, other.rootNode_);
        std::swap(indices_, other.indices_);
        return *this;
    }

    BvhNode&			root() { return *rootNode_; }
    const BvhNode&		root() const { return *rootNode_; }

    void				save(std::ostream& os);

	uint32_t			getIndex(uint32_t index) const { return indices_[index]; }

private:


    SplitMode						mode_;
    std::unique_ptr<BvhNode>		rootNode_;
    
	std::vector<uint32_t>			indices_; // triangle index list that will be sorted during BVH construction
};


}