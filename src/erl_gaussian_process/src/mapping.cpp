#include "erl_gaussian_process/mapping.hpp"

#include <cmath>

namespace erl::gaussian_process {
    std::shared_ptr<Mapping>
    Mapping::Create() {
        return std::shared_ptr<Mapping>(new Mapping());
    }

    std::shared_ptr<Mapping>
    Mapping::Create(std::shared_ptr<Setting> setting) {
        return std::shared_ptr<Mapping>(new Mapping(std::move(setting)));
    }

    Mapping::Mapping()
        : Mapping(std::make_shared<Setting>()) {}

    Mapping::Mapping(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        switch (m_setting_->type) {
            case Type::kIdentity: {
                map = [](const double x) { return x; };
                inv = map;
                break;
            }
            case Type::kInverse: {
                map = [](const double x) { return 1. / x; };
                inv = map;
                break;
            }
            case Type::kInverseSqrt: {
                map = [](const double x) { return 1. / std::sqrt(x); };
                inv = [](const double y) { return 1. / (y * y); };
                break;
            }
            case Type::kExp: {
                map = [&](const double x) { return std::exp(-m_setting_->scale * x); };
                inv = [&](const double y) { return -std::log(y) / m_setting_->scale; };
                break;
            }
            case Type::kLog: {
                map = [&](const double x) { return std::log(m_setting_->scale * x); };
                inv = [&](const double y) { return std::exp(y) / m_setting_->scale; };
                break;
            }
            case Type::kTanh: {
                map = [&](const double x) { return std::tanh(m_setting_->scale * x); };
                inv = [&](const double y) { return std::atanh(y) / m_setting_->scale; };
                break;
            }
            case Type::kSigmoid: {
                map = [&](const double x) { return 1. / (1. + std::exp(-m_setting_->scale * x)); };
                inv = [&](const double y) {
                    if (y >= 1.) { return std::numeric_limits<double>::infinity() / m_setting_->scale; }
                    if (y <= 0.) { return -std::numeric_limits<double>::infinity() / m_setting_->scale; }
                    return std::log(y / (1. - y)) / m_setting_->scale;
                };
                break;
            }
            case Type::kUnknown:
            default:
                throw std::logic_error("Mapping type is kUnknown, which is unexpected.");
        }
    }
}  // namespace erl::gaussian_process
