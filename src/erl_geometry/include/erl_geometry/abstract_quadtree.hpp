#pragma once

#include "nd_tree_setting.hpp"

#include "erl_common/string_utils.hpp"

#include <map>
#include <memory>
#include <string>

namespace erl::geometry {

    /**
     * AbstractQuadtree is a base class for all quadtree implementations. It provides a common interface for factory pattern and file I/O.
     */
    class AbstractQuadtree {
    protected:
        std::shared_ptr<NdTreeSetting> m_setting_ = std::make_shared<NdTreeSetting>();
        inline static std::map<std::string, std::shared_ptr<AbstractQuadtree>> s_class_id_mapping_ = {};  // cppcheck-suppress unusedStructMember
        inline static const std::string sk_FileHeader_ = "# erl::geometry::AbstractQuadtree";             // cppcheck-suppress unusedStructMember

    public:
        AbstractQuadtree() = delete;  // no default constructor

        explicit AbstractQuadtree(const std::shared_ptr<NdTreeSetting>& setting)
            : m_setting_(setting) {
            ERL_DEBUG_WARN_ONCE_COND(
                typeid(*this) != typeid(AbstractQuadtree) && s_class_id_mapping_.find(GetTreeType()) == s_class_id_mapping_.end(),
                "Tree type {} not registered, do you forget to use ERL_REGISTER_QUADTREE({})?",
                GetTreeType(),
                GetTreeType());
        }

        AbstractQuadtree(const AbstractQuadtree& other) = default;
        AbstractQuadtree&
        operator=(const AbstractQuadtree& other) = default;
        AbstractQuadtree(AbstractQuadtree&& other) = default;
        AbstractQuadtree&
        operator=(AbstractQuadtree&& other) = default;

        virtual ~AbstractQuadtree() = default;

        //-- factory pattern
        /**
         * returns actual class name as string for identification
         * @return The type of the tree.
         */
        [[nodiscard]] std::string
        GetTreeType() const {
            return demangle(typeid(*this).name());
        }

        /**
         * Implemented by derived classes to create a new tree of the same type.
         * @return A new tree of the same type.
         */
        [[nodiscard]] virtual std::shared_ptr<AbstractQuadtree>
        Create() const = 0;

        /**
         * Create a new tree of the given type.
         * @param tree_id
         * @return
         */
        static std::shared_ptr<AbstractQuadtree>
        CreateTree(const std::string& tree_id);

        static void
        RegisterTreeType(const std::shared_ptr<AbstractQuadtree>& tree) {
            s_class_id_mapping_[tree->GetTreeType()] = tree;
        }

        //-- setting
        /**
         * Get the setting of the tree.
         * @tparam T The type of the setting.
         * @return
         */
        template<typename T>
        std::shared_ptr<T>
        GetSetting() const {
            return std::reinterpret_pointer_cast<T>(m_setting_);
        }

        /**
         * This function should be called when the setting is changed.
         */
        virtual void
        ApplySetting() = 0;

        void
        ReadSetting(std::istream& s) const {
            std::streamsize len;
            s.read(reinterpret_cast<char*>(&len), sizeof(std::size_t));
            std::string yaml_str(len, '\0');
            s.read(yaml_str.data(), len);
            m_setting_->FromYamlString(yaml_str);
        }

        void
        WriteSetting(std::ostream& s) const {
            const std::string yaml_str = m_setting_->AsYamlString();
            const auto len = static_cast<std::streamsize>(yaml_str.size());
            s.write(reinterpret_cast<const char*>(&len), sizeof(std::size_t));
            s.write(yaml_str.data(), len);
        }

        //-- comparison
        [[nodiscard]] virtual bool
        operator==(const AbstractQuadtree& other) const = 0;

        [[nodiscard]] bool
        operator!=(const AbstractQuadtree& other) const {
            return !(*this == other);
        }

        //-- get tree information
        [[nodiscard]] uint32_t
        GetTreeDepth() const {
            return m_setting_->tree_depth;
        }

        [[nodiscard]] double
        GetResolution() const {
            return m_setting_->resolution;
        }

        [[nodiscard]] virtual std::size_t
        GetSize() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsage() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsagePerNode() const = 0;
        virtual void
        GetMetricMin(double& x, double& y) = 0;
        virtual void
        GetMetricMin(double& x, double& y) const = 0;
        virtual void
        GetMetricMax(double& x, double& y) = 0;
        virtual void
        GetMetricMax(double& x, double& y) const = 0;
        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& max_x, double& max_y) = 0;
        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& max_x, double& max_y) const = 0;
        virtual void
        GetMetricSize(double& x, double& y) = 0;
        virtual void
        GetMetricSize(double& x, double& y) const = 0;

        //-- IO
        virtual void
        Clear() = 0;
        virtual void
        Prune() = 0;
        /**
         * Write the tree as raw data to a file.
         * @param filename
         * @return
         */
        [[nodiscard]] bool
        Write(const std::string& filename) const;
        /**
         * Write the tree as raw data to a stream.
         * @param s
         * @return
         */
        [[nodiscard]] std::ostream&
        Write(std::ostream& s) const;

    protected:
        /**
         * Write all nodes to the output stream (without file header) for a created tree. Pruning the tree first produces smaller files and faster loading.
         * @param s
         * @return
         */
        virtual std::ostream&
        WriteData(std::ostream& s) const = 0;

    public:
        /**
         * Read an octree from a file and cast it to the given type.
         * @tparam T
         * @param filename
         * @return may return nullptr if the cast fails
         */
        template<typename T>
        static std::enable_if_t<std::is_base_of_v<AbstractQuadtree, T>, std::shared_ptr<T>>
        ReadAs(const std::string& filename) {
            return std::dynamic_pointer_cast<T>(Read(filename));
        }

        /**
         * Generic read function to read an octree from a file.
         * @param filename
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractQuadtree>
        Read(const std::string& filename);
        /**
         * Generic read function to read an octree from a stream.
         * @param s
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractQuadtree>
        Read(std::istream& s);

    protected:
        /**
         * Read all nodes from the input steam (without file header) for a created tree.
         */
        virtual std::istream&
        ReadData(std::istream& s) = 0;

    public:
        /**
         * Load the tree data from a file, the tree type in the file has to match the actual tree type.
         * @param filename
         * @return
         */
        bool
        LoadData(const std::string& filename);

        /**
         * Load the tree data from a stream, the tree type in the file has to match the actual tree type.
         * @param s
         * @return
         */
        bool
        LoadData(std::istream& s);

    protected:
        static bool
        ReadHeader(std::istream& s, std::string& tree_id, uint32_t& size);
    };

#define ERL_REGISTER_QUADTREE(tree_type)                         \
    inline const volatile bool kRegistered##tree_type = []() {   \
        auto tree = std::make_shared<tree_type>();               \
        tree->ClearKeyRays();                                    \
        erl::geometry::AbstractQuadtree::RegisterTreeType(tree); \
        ERL_DEBUG(#tree_type " is registered.");                 \
        return true;                                             \
    }()
}  // namespace erl::geometry
