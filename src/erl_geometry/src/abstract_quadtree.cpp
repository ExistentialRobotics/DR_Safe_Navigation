#include "erl_geometry/abstract_quadtree.hpp"

#include "erl_common/logging.hpp"

#include <filesystem>
#include <fstream>

namespace erl::geometry {

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::CreateTree(const std::string &tree_id) {
        const auto it = s_class_id_mapping_.find(tree_id);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARN("Unknown Quadtree type: {}", tree_id);
            return nullptr;
        }

        return it->second->Create();
    }

    bool
    AbstractQuadtree::Write(const std::string &filename) const {
        ERL_INFO("Writing quadtree to file: {}", std::filesystem::absolute(filename));
        std::ofstream file(filename, std::ios_base::out | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        (void) Write(file);
        file.close();
        ERL_INFO("Successfully wrote Quadtree of type {}, size {}", this->GetTreeType(), this->GetSize());
        return true;
    }

    static const std::string kFileHeader = "# erl::geometry::AbstractQuadtree";

    std::ostream &
    AbstractQuadtree::Write(std::ostream &s) const {
        // write header
        s << kFileHeader << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << this->GetTreeType() << std::endl
          << "size " << this->GetSize() << std::endl
          << "setting" << std::endl;
        this->WriteSetting(s);
        s << "data" << std::endl;
        // write the actual tree data
        return this->WriteData(s);
    }

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::Read(const std::string &filename) {
        ERL_INFO("Reading octree from file: {}", std::filesystem::absolute(filename));
        std::ifstream file(filename, std::ios_base::in | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return nullptr;
        }

        auto tree = Read(file);
        file.close();
        return tree;
    }

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::Read(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return nullptr;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, kFileHeader.length(), kFileHeader) != 0) {
            ERL_WARN("First line of Quadtree file header does not start with \"{}\"", kFileHeader);
            return nullptr;
        }

        std::string tree_id;
        uint32_t size;
        if (!ReadHeader(s, tree_id, size)) { return nullptr; }

        ERL_DEBUG("Reading Quadtree of type {}, size {}", tree_id, size);
        auto tree = CreateTree(tree_id);
        if (!tree) { return nullptr; }
        tree->ReadSetting(s);
        tree->ApplySetting();

        std::getline(s, line);  // check if the next line is "data"
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Error reading Quadtree, expected 'data' section");
            return nullptr;
        }

        // read the actual tree data
        if (size > 0) { tree->ReadData(s); }
        ERL_INFO("Done ({} nodes).", tree->GetSize());
        return tree;
    }

    bool
    AbstractQuadtree::LoadData(const std::string &filename) {
        ERL_INFO("Loading data from file: {}", std::filesystem::absolute(filename));
        std::ifstream s(filename, std::ios_base::in | std::ios_base::binary);
        if (!s.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }
        const bool success = LoadData(s);
        s.close();
        return success;
    }

    bool
    AbstractQuadtree::LoadData(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return false;
        }

        std::string line;
        std::getline(s, line);
        if (line.compare(0, kFileHeader.length(), kFileHeader) != 0) {  // check if the first line is valid
            ERL_WARN("First line of Octree file header does not start with \"{}\"", kFileHeader.c_str());
            return false;
        }

        std::string tree_id;
        uint32_t size;
        if (!ReadHeader(s, tree_id, size)) { return false; }

        if (tree_id != GetTreeType()) {
            ERL_WARN("Error reading Quadtree header, ID does not match: {} != {}", tree_id.c_str(), GetTreeType().c_str());
            return false;
        }
        Clear();
        ReadSetting(s);
        ApplySetting();

        std::getline(s, line);  // check if the next line is "data"
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: {}", line.c_str());
            return false;
        }
        if (size > 0) { this->ReadData(s); }
        ERL_DEBUG("Done ({} nodes).", this->GetSize());
        return GetSize() == size;
    }

    bool
    AbstractQuadtree::ReadHeader(std::istream &s, std::string &tree_id, uint32_t &size) {
        // initialize output variables
        tree_id = "";
        size = 0;
        // read header
        std::string token;
        bool header_read = false;
        while (s.good() && !header_read) {
            s >> token;
            if (token == "setting") {  // reach the setting section
                header_read = true;    // header read successfully
                // skip forward until end of line
                char c;
                do { c = static_cast<char>(s.get()); } while (s.good() && c != '\n');
            } else if (token.compare(0, 1, "#") == 0) {
                // comment line, skip forward until end of line
                char c;
                do { c = static_cast<char>(s.get()); } while (s.good() && c != '\n');
            } else if (token == "id") {
                s >> tree_id;
            } else if (token == "size") {
                s >> size;
            } else {
                ERL_WARN("Unknown keyword in Quadtree header, skipping: {}", token.c_str());
                char c;
                do { c = static_cast<char>(s.get()); } while (s.good() && c != '\n');
            }
        }

        if (!header_read) {
            ERL_WARN("Error reading Quadtree header");
            return false;
        }

        if (tree_id.empty()) {
            ERL_WARN("Error reading Quadtree header, ID not set");
            return false;
        }

        return true;
    }
}  // namespace erl::geometry
