
#include "Bvh.hpp"
#include "filesaves.hpp"

#include <algorithm>


namespace FW {


Bvh::Bvh() { }


// reconstruct from a file
Bvh::Bvh(std::istream& is)
{
    // Load file header.
    fileload(is, mode_);
    Statusbar nodeInfo("Loading nodes", 0);
    Loader loader(is, nodeInfo);

    // Load elements.
    {
        size_t size;
        fileload(is, size);

        for(size_t i = 0; i < size; ++i) {
            uint32_t idx;
			loader(idx);
            indices_.push_back(idx);
        }
    }

    // Load the rest.
    rootNode_.reset(new BvhNode(loader));
}

void Bvh::save(std::ostream& os) {
    // Save file header.
    filesave(os, mode_);
    Statusbar nodeInfo("Saving nodes", 0);
    Saver saver(os, nodeInfo);

    // Save elements.
    {
        filesave(os, (size_t)indices_.size());

        for(auto& i : indices_) {
            saver(i);
        }
    }

    // Save the rest.
    rootNode_->save(saver);
}

}
