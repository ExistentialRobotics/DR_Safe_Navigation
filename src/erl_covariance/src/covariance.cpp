#include "erl_covariance/covariance.hpp"
// include all covariance headers to register them
#include "erl_covariance/matern32.hpp"            // ReSharper disable once CppUnusedIncludeDirective
#include "erl_covariance/ornstein_uhlenbeck.hpp"  // ReSharper disable once CppUnusedIncludeDirective

namespace erl::covariance {
    std::shared_ptr<Covariance>
    Covariance::CreateCovariance(const std::string &covariance_type) {
        const auto it = s_class_id_mapping_.find(covariance_type);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARN("Unknown covariance type: {}", covariance_type);
            return nullptr;
        }
        return it->second->Create();
    }

    void
    Covariance::RegisterCovarianceType(const std::shared_ptr<Covariance> &covariance) {
        s_class_id_mapping_[covariance->GetCovarianceType()] = covariance;
    }

}  // namespace erl::covariance
