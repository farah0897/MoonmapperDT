// Copyright 2026 MoonMapper contributors
//
// Gazebo Sim 8 (Harmonic) system plugin: synthetic Triad spectroscopy sensors.
//
// Level 1 design goals:
//  - Two independent sensors (triad_sensor_1, triad_sensor_2)
//  - Raycast down from each sensor link to identify what is underneath
//  - Map hit name -> material label -> synthetic spectrum vector (lookup table)
//  - Add optional Gaussian noise
//  - Publish ROS 2 message moonmapper_msgs/msg/TriadSpectrum

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/RaycastData.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <rclcpp/rclcpp.hpp>

#include <moonmapper_msgs/msg/triad_spectrum.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace moonmapper
{

struct HitRule
{
  std::string matchLower;
  std::string material;
};

struct ZoneRule
{
  double xmin{0}, xmax{0}, ymin{0}, ymax{0};
  std::string material;
};

static std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static bool EndsWith(const std::string &s, const std::string &suffix)
{
  if (suffix.size() > s.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

class TriadSpectroscopy
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[TriadSpectroscopy] Plugin must be attached to a <model>\n";
      return;
    }

    this->enable_ = _sdf->Get<bool>("enable", true).first;
    this->update_rate_hz_ = _sdf->Get<double>("update_rate_hz", 20.0).first;
    this->max_distance_ = _sdf->Get<double>("max_distance", 0.10).first;
    this->noise_stddev_ = _sdf->Get<double>("noise_stddev", 0.01).first;
    this->channels_count_ = static_cast<size_t>(
      std::max(1, _sdf->Get<int>("channels", 18).first));

    // AS7265x-inspired measurement controls (datasheet):
    // gain: 0=1x, 1=3.7x, 2=16x, 3=64x
    // integration_time_reg: value * 2.8ms
    // bank_mode: 2 uses both banks -> ~2x integration time for full 6 channels
    // interval_mult: sampling interval multiplier (ATINTRVL)
    this->as7265x_gain_ = std::clamp(_sdf->Get<int>("as7265x_gain", 2).first, 0, 3);
    this->as7265x_int_reg_ = std::clamp(_sdf->Get<int>("as7265x_integration_time_reg", 20).first, 1, 255);
    this->as7265x_bank_mode_ = std::clamp(_sdf->Get<int>("as7265x_bank_mode", 2).first, 0, 3);
    this->as7265x_interval_mult_ = std::clamp(_sdf->Get<int>("as7265x_interval_mult", 1).first, 1, 255);

    this->sensor1_link_name_ = _sdf->Get<std::string>(
      "sensor1_link", std::string{"triad_sensor_1_link"}).first;
    this->sensor2_link_name_ = _sdf->Get<std::string>(
      "sensor2_link", std::string{"triad_sensor_2_link"}).first;

    // In Gazebo Sim it's common that fixed joints are lumped and the child links
    // never exist in physics. For Level 1 robustness we raycast from base_link
    // with per-sensor offsets (defaults taken from your URDF joint origins).
    this->base_link_name_ = _sdf->Get<std::string>(
      "base_link", std::string{"base_link"}).first;

    this->s1_xyz_ = _sdf->Get<gz::math::Vector3d>(
      "sensor1_xyz",
      gz::math::Vector3d(-0.091911, 0.046088, -0.043464)).first;
    this->s2_xyz_ = _sdf->Get<gz::math::Vector3d>(
      "sensor2_xyz",
      gz::math::Vector3d(-0.091834, -0.048139, -0.043518)).first;

    // Optional orientation offsets (RPY in radians). Defaults zero.
    this->s1_rpy_ = _sdf->Get<gz::math::Vector3d>("sensor1_rpy", gz::math::Vector3d::Zero).first;
    this->s2_rpy_ = _sdf->Get<gz::math::Vector3d>("sensor2_rpy", gz::math::Vector3d::Zero).first;

    this->topic1_ = _sdf->Get<std::string>(
      "topic1", std::string{"/triad_sensor_1/spectrum"}).first;
    this->topic2_ = _sdf->Get<std::string>(
      "topic2", std::string{"/triad_sensor_2/spectrum"}).first;

    this->map_yaml_ = _sdf->Get<std::string>(
      "material_map_yaml", std::string{""}).first;

    if (!this->map_yaml_.empty()) {
      this->LoadMaterialMap(this->map_yaml_);
    } else {
      this->LoadDefaultMaterialMap();
    }

    // ROS 2 node inside the plugin
    if (!rclcpp::ok()) {
      // Gazebo sim may launch without a ROS context; initialize a private one.
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
      this->owns_rclcpp_context_ = true;
    }

    this->ros_node_ = std::make_shared<rclcpp::Node>(
      "triad_spectroscopy_gz",
      rclcpp::NodeOptions().use_intra_process_comms(false));

    // Use RELIABLE QoS so ros2 topic echo/bag work by default.
    // (SensorDataQoS is best-effort and often appears as "no data" in CLI tools.)
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    this->pub1_ = this->ros_node_->create_publisher<moonmapper_msgs::msg::TriadSpectrum>(
      this->topic1_, qos);
    this->pub2_ = this->ros_node_->create_publisher<moonmapper_msgs::msg::TriadSpectrum>(
      this->topic2_, qos);

    gzmsg << "[TriadSpectroscopy] Configured for model '"
          << this->model_.Name(_ecm) << "'\n"
          << "  base_link=" << this->base_link_name_ << "\n"
          << "  sensor1_frame=" << this->sensor1_link_name_ << " topic1=" << this->topic1_ << "\n"
          << "  sensor2_frame=" << this->sensor2_link_name_ << " topic2=" << this->topic2_ << "\n"
          << "  update_rate_hz=" << this->update_rate_hz_
          << " max_distance=" << this->max_distance_
          << " noise_stddev=" << this->noise_stddev_
          << " channels=" << this->channels_count_ << "\n";

    gzmsg << "[TriadSpectroscopy] AS7265x model: gain=" << this->as7265x_gain_
          << " int_reg=" << this->as7265x_int_reg_
          << " bank_mode=" << this->as7265x_bank_mode_
          << " interval_mult=" << this->as7265x_interval_mult_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->enable_ || _info.paused) {
      return;
    }

    // Cache base link entity; raycasts are requested on this entity using per-sensor offsets.
    if (this->base_link_ == gz::sim::kNullEntity) {
      this->base_link_ = this->model_.LinkByName(_ecm, this->base_link_name_);
      if (this->base_link_ == gz::sim::kNullEntity) {
        this->base_link_ = this->FindLinkBySuffix(_ecm, this->base_link_name_);
      }
      // Gazebo often chooses a different canonical/root link than URDF's base_link
      // (e.g. base_footprint) due to fixed-joint lumping. For robustness, fall
      // back to the model's canonical link so we still raycast and publish.
      if (this->base_link_ == gz::sim::kNullEntity) {
        this->base_link_ = this->model_.CanonicalLink(_ecm);
      }
      if (this->base_link_ == gz::sim::kNullEntity && !this->warned_missing_base_) {
        gzwarn << "[TriadSpectroscopy] Could not find base link '" << this->base_link_name_
               << "' in model yet (no publishing until found)\n";
        this->warned_missing_base_ = true;
      }
      if (this->base_link_ != gz::sim::kNullEntity) {
        gzmsg << "[TriadSpectroscopy] Using raycast base link entity '"
              << this->LinkName(_ecm, this->base_link_) << "'\n";
      }
    }

    // Attach raycast request component on the base link (physics system fills results).
    this->EnsureRaycastComponent(_ecm, this->base_link_);

    // Throttle with sim time
    const auto simNs = std::chrono::nanoseconds(_info.simTime);
    const auto period = this->EffectiveSamplePeriod();
    if ((simNs - this->last_update_sim_ns_) < period) {
      return;
    }
    this->last_update_sim_ns_ = simNs;

    // Update ray definitions: 2 rays on the same base entity (one per sensor offset).
    this->UpdateRaysOnBase(_ecm);
    this->publish_due_ = true;
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->enable_ || _info.paused) {
      return;
    }
    if (!this->ros_node_) {
      return;
    }

    // Publish only when rays were updated in PreUpdate (avoid sim-step publish rate).
    if (this->publish_due_) {
      this->PublishFromBase(_info, _ecm);
      this->publish_due_ = false;
    }

    // Allow ROS callbacks if ever added later (kept cheap).
    rclcpp::spin_some(this->ros_node_);
  }

  ~TriadSpectroscopy() override
  {
    if (this->owns_rclcpp_context_) {
      rclcpp::shutdown();
    }
  }

private:
  // AS7265x channel order in datasheet ATDATA/ATCDATA:
  //   R,S,T,U,V,W, G,H,I,J,K,L, A,B,C,D,E,F
  // Center wavelengths (nm) per datasheet.
  static constexpr std::array<double, 18> kAs7265xWavelengthNm = {
    610, 680, 730, 760, 810, 860,
    560, 585, 645, 705, 900, 940,
    410, 435, 460, 485, 510, 535
  };

  double GainMultiplier() const
  {
    switch (this->as7265x_gain_) {
      case 0: return 1.0;
      case 1: return 3.7;
      case 2: return 16.0;
      case 3: return 64.0;
      default: return 16.0;
    }
  }

  std::chrono::nanoseconds IntegrationTimeNs() const
  {
    const double tint_ms = static_cast<double>(this->as7265x_int_reg_) * 2.8;
    const bool twoBanks = (this->as7265x_bank_mode_ == 2 || this->as7265x_bank_mode_ == 3);
    const double effective_ms =
      tint_ms * (twoBanks ? 2.0 : 1.0) * static_cast<double>(this->as7265x_interval_mult_);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(effective_ms));
  }

  std::chrono::nanoseconds EffectiveSamplePeriod() const
  {
    if (this->update_rate_hz_ > 0.0) {
      const auto p = std::chrono::duration<double>(1.0 / this->update_rate_hz_);
      return std::chrono::duration_cast<std::chrono::nanoseconds>(p);
    }
    return this->IntegrationTimeNs();
  }

  std::vector<float> As7265xCountsFromSpectrum(const std::vector<float> &spectrum01) const
  {
    const double gain = this->GainMultiplier();
    const double tint_ms = std::chrono::duration<double, std::milli>(this->IntegrationTimeNs()).count();

    // Dark baseline: datasheet suggests ~5 counts @ gain=64, tint~165ms.
    // Keep a small baseline that increases mildly with gain and integration.
    const double dark = 1.0 + 0.02 * gain + 0.01 * (tint_ms / 10.0);

    // Convert reflectance-like values (0..1) into raw-count-like values.
    // Conservative scaling avoids constant saturation while keeping good dynamic range.
    const double scale = 120.0;
    const double tintNorm = std::max(1.0, tint_ms / 10.0);

    std::vector<float> out;
    out.resize(this->channels_count_, 0.0f);
    for (size_t i = 0; i < out.size(); ++i) {
      const double r = (i < spectrum01.size())
        ? std::clamp(static_cast<double>(spectrum01[i]), 0.0, 1.0)
        : 0.1;
      double c = dark + scale * r * gain * tintNorm;
      c = std::clamp(c, 0.0, 65535.0);
      out[i] = static_cast<float>(c);
    }
    return out;
  }

  void EnsureRaycastComponent(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _link)
  {
    if (_link == gz::sim::kNullEntity) {
      return;
    }
    if (!_ecm.Component<gz::sim::components::RaycastData>(_link)) {
      _ecm.CreateComponent(_link, gz::sim::components::RaycastData());
    }
  }

  gz::sim::Entity FindLinkBySuffix(gz::sim::EntityComponentManager &_ecm,
                                  const std::string &unscopedLinkName) const
  {
    // Prefer iterating model links directly; this is more robust across
    // scoped naming and component layouts.
    for (const auto &link : this->model_.Links(_ecm)) {
      const auto *name = _ecm.Component<gz::sim::components::Name>(link);
      if (!name) {
        continue;
      }
      const auto &n = name->Data();
      if (n == unscopedLinkName || EndsWith(n, "::" + unscopedLinkName)) {
        return link;
      }
    }
    return gz::sim::kNullEntity;
  }

  void UpdateRaysOnBase(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->base_link_ == gz::sim::kNullEntity) {
      return;
    }
    auto *rayComp = _ecm.Component<gz::sim::components::RaycastData>(this->base_link_);
    if (!rayComp) {
      return;
    }

    // RayInfo is expressed in ENTITY frame (see RaycastData.hh).
    // We cast along local -Z so "down" follows the sensor mounting.
    auto data = rayComp->Data();
    data.rays.clear();

    const auto s1Rot = gz::math::Quaterniond(this->s1_rpy_.X(), this->s1_rpy_.Y(), this->s1_rpy_.Z());
    const auto s2Rot = gz::math::Quaterniond(this->s2_rpy_.X(), this->s2_rpy_.Y(), this->s2_rpy_.Z());

    const auto s1Dir = s1Rot.RotateVectorReverse(gz::math::Vector3d::UnitZ);
    const auto s2Dir = s2Rot.RotateVectorReverse(gz::math::Vector3d::UnitZ);

    gz::sim::components::RayInfo ray1;
    ray1.start = this->s1_xyz_;
    ray1.end = this->s1_xyz_ - s1Dir * this->max_distance_;
    data.rays.push_back(ray1);

    gz::sim::components::RayInfo ray2;
    ray2.start = this->s2_xyz_;
    ray2.end = this->s2_xyz_ - s2Dir * this->max_distance_;
    data.rays.push_back(ray2);

    _ecm.SetComponentData<gz::sim::components::RaycastData>(this->base_link_, data);
  }

  void PublishFromBase(const gz::sim::UpdateInfo &_info,
                       const gz::sim::EntityComponentManager &_ecm)
  {
    if (this->base_link_ == gz::sim::kNullEntity) {
      return;
    }
    const auto *rayComp = _ecm.Component<gz::sim::components::RaycastData>(this->base_link_);
    if (!rayComp) {
      return;
    }

    const auto data = rayComp->Data();

    auto publishOne = [&](size_t idx,
                          const std::string &sensorFrame,
                          const std::string &sensorName,
                          rclcpp::Publisher<moonmapper_msgs::msg::TriadSpectrum> &pub)
    {
      std::string hitName;
      double confidence = 0.0;
      std::string material = "unknown";
      gz::math::Vector3d hitPointWorld = gz::math::Vector3d::Zero;

      if (idx < data.results.size()) {
        const auto &res = data.results[idx];
        const auto basePose = gz::sim::worldPose(this->base_link_, _ecm);
        hitPointWorld = basePose.Pos() + basePose.Rot().RotateVector(res.point);

        material = this->MaterialForHit(hitName);
        if (material == "unknown") {
          material = this->MaterialForPoint(hitPointWorld);
        }
        confidence = (material == "unknown") ? 0.2 : 0.8;
      }

      // 1) material spectrum (0..1-ish) in AS7265x channel order
      const auto spectrum01 = this->SpectrumForMaterial(material);
      // 2) convert to AS7265x-ish "raw counts" (16-bit domain)
      auto channels = this->As7265xCountsFromSpectrum(spectrum01);
      // 3) add gaussian noise in "counts"
      this->ApplyNoise(channels);
      // 4) normalized view (0..1) for convenience
      auto normalized = this->Normalize(channels);

      moonmapper_msgs::msg::TriadSpectrum msg;
      msg.header.stamp = rclcpp::Time(_info.simTime.count(), RCL_ROS_TIME);
      msg.header.frame_id = sensorFrame;
      msg.sensor_name = sensorName;
      msg.hit_name = hitName;
      msg.material_label = material;
      msg.confidence = static_cast<float>(confidence);
      msg.channels = std::move(channels);
      msg.channels_normalized = std::move(normalized);
      pub.publish(std::move(msg));
    };

    publishOne(0, this->sensor1_link_name_, "triad_sensor_1", *this->pub1_);
    publishOne(1, this->sensor2_link_name_, "triad_sensor_2", *this->pub2_);
  }

  std::string LinkName(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _e) const
  {
    const auto *name = _ecm.Component<gz::sim::components::Name>(_e);
    if (!name) {
      return {};
    }
    return name->Data();
  }

  std::string MaterialForHit(const std::string &hitName) const
  {
    const auto hitLower = ToLower(hitName);
    for (const auto &rule : this->hit_rules_) {
      if (!rule.matchLower.empty() && hitLower.find(rule.matchLower) != std::string::npos) {
        return rule.material;
      }
    }
    return "unknown";
  }

  std::string MaterialForPoint(const gz::math::Vector3d &pWorld) const
  {
    for (const auto &z : this->zone_rules_) {
      if (pWorld.X() >= z.xmin && pWorld.X() <= z.xmax &&
          pWorld.Y() >= z.ymin && pWorld.Y() <= z.ymax)
      {
        return z.material;
      }
    }
    return "unknown";
  }

  std::vector<float> SpectrumForMaterial(const std::string &material) const
  {
    auto it = this->materials_.find(material);
    if (it == this->materials_.end()) {
      it = this->materials_.find("unknown");
    }
    std::vector<float> out;
    out.resize(this->channels_count_, 0.1f);
    if (it != this->materials_.end()) {
      const auto &v = it->second;
      for (size_t i = 0; i < out.size() && i < v.size(); ++i) {
        out[i] = v[i];
      }
    }
    return out;
  }

  void ApplyNoise(std::vector<float> &channels)
  {
    if (this->noise_stddev_ <= 0.0) {
      return;
    }
    for (auto &c : channels) {
      c += static_cast<float>(this->noise_dist_(this->rng_) * this->noise_stddev_);
      if (c < 0.0f) c = 0.0f;
    }
  }

  std::vector<float> Normalize(const std::vector<float> &channels) const
  {
    float maxv = 0.0f;
    for (const auto &c : channels) {
      maxv = std::max(maxv, c);
    }
    std::vector<float> out;
    out.reserve(channels.size());
    if (maxv <= 1e-9f) {
      out.assign(channels.size(), 0.0f);
      return out;
    }
    for (const auto &c : channels) {
      out.push_back(c / maxv);
    }
    return out;
  }

  void LoadDefaultMaterialMap()
  {
    // Minimal defaults; can be overridden by YAML.
    this->materials_.clear();
    this->materials_["unknown"] = std::vector<float>(this->channels_count_, 0.10f);
    this->materials_["regolith"] = std::vector<float>(this->channels_count_, 0.18f);
    this->materials_["basalt"] = std::vector<float>(this->channels_count_, 0.08f);
    this->materials_["metal"] = std::vector<float>(this->channels_count_, 0.45f);
    this->materials_["ice"] = std::vector<float>(this->channels_count_, 0.25f);
    this->hit_rules_.clear();
  }

  void LoadMaterialMap(const std::string &path)
  {
    try {
      const YAML::Node root = YAML::LoadFile(path);
      this->materials_.clear();
      this->hit_rules_.clear();

      const auto mats = root["materials"];
      if (mats && mats.IsMap()) {
        for (const auto it : mats) {
          const auto label = it.first.as<std::string>();
          const auto channels = it.second["channels"];
          std::vector<float> vec;
          if (channels && channels.IsSequence()) {
            for (const auto &x : channels) {
              vec.push_back(x.as<float>());
            }
          }
          this->materials_[label] = std::move(vec);
        }
      }

      const auto rules = root["hit_name_rules"];
      if (rules && rules.IsSequence()) {
        for (const auto &r : rules) {
          HitRule rule;
          rule.matchLower = ToLower(r["match"].as<std::string>(""));
          rule.material = r["material"].as<std::string>("unknown");
          this->hit_rules_.push_back(std::move(rule));
        }
      }

      const auto zones = root["zones_xy"];
      if (zones && zones.IsSequence()) {
        for (const auto &z : zones) {
          ZoneRule zr;
          zr.xmin = z["xmin"].as<double>(0.0);
          zr.xmax = z["xmax"].as<double>(0.0);
          zr.ymin = z["ymin"].as<double>(0.0);
          zr.ymax = z["ymax"].as<double>(0.0);
          zr.material = z["material"].as<std::string>("unknown");
          this->zone_rules_.push_back(std::move(zr));
        }
      }

      if (this->materials_.find("unknown") == this->materials_.end()) {
        this->materials_["unknown"] = std::vector<float>(this->channels_count_, 0.10f);
      }

      gzmsg << "[TriadSpectroscopy] Loaded material map from '" << path
            << "' (materials=" << this->materials_.size()
            << ", hit_rules=" << this->hit_rules_.size()
            << ", zones_xy=" << this->zone_rules_.size() << ")\n";
    } catch (const std::exception &e) {
      gzwarn << "[TriadSpectroscopy] Failed to load YAML '" << path
             << "': " << e.what() << " (using defaults)\n";
      this->LoadDefaultMaterialMap();
    }
  }

private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  bool enable_{true};

  std::string base_link_name_;
  gz::sim::Entity base_link_{gz::sim::kNullEntity};
  bool warned_missing_base_{false};

  std::string sensor1_link_name_;
  std::string sensor2_link_name_;
  gz::math::Vector3d s1_xyz_{0, 0, 0};
  gz::math::Vector3d s2_xyz_{0, 0, 0};
  gz::math::Vector3d s1_rpy_{0, 0, 0};
  gz::math::Vector3d s2_rpy_{0, 0, 0};

  double update_rate_hz_{20.0};
  double max_distance_{0.10};
  double noise_stddev_{0.01};
  size_t channels_count_{18};
  std::chrono::nanoseconds last_update_sim_ns_{0};
  bool publish_due_{false};

  int as7265x_gain_{2};
  int as7265x_int_reg_{20};
  int as7265x_bank_mode_{2};
  int as7265x_interval_mult_{1};

  std::string topic1_;
  std::string topic2_;

  std::string map_yaml_;
  std::unordered_map<std::string, std::vector<float>> materials_;
  std::vector<HitRule> hit_rules_;
  std::vector<ZoneRule> zone_rules_;

  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> noise_dist_{0.0, 1.0};

  bool owns_rclcpp_context_{false};
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<moonmapper_msgs::msg::TriadSpectrum>::SharedPtr pub1_;
  rclcpp::Publisher<moonmapper_msgs::msg::TriadSpectrum>::SharedPtr pub2_;
};

}  // namespace moonmapper

GZ_ADD_PLUGIN(
  moonmapper::TriadSpectroscopy,
  gz::sim::System,
  moonmapper::TriadSpectroscopy::ISystemConfigure,
  moonmapper::TriadSpectroscopy::ISystemPreUpdate,
  moonmapper::TriadSpectroscopy::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(moonmapper::TriadSpectroscopy, "moonmapper::TriadSpectroscopy")

