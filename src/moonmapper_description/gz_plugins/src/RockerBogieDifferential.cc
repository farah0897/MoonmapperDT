// Copyright 2026 MoonMapper contributors
//
// Gazebo Sim 8 (Harmonic) system plugin: RockerBogieDifferential.
//
// Dette pluginet gjoer TO ting i samme PreUpdate-loop:
//
// 1) Haandhever den "virtuelle" differensial-bjelken mellom venstre
//    og hoegre rocker ved aa legge PD-moment paa summen av vinklene:
//
//        tau_L = tau_R = -kp*(qL + qR - target_sum) - kd*(dqL + dqR)
//
//    Dette gir den anti-symmetriske oppfoerselen en ekte rocker-bogie
//    walking-beam ville hatt.
//
// 2) PD-posisjonskontrollerer de tre "kosmetiske" leddene
//    (hengsel_diff_L_joint, hengsel_diff_R_joint, rocker_bogie_diff_joint)
//    slik at de foelger synkront med rockerne i henhold til en affin
//    kinematisk relasjon:
//
//        q_diff*   = k_diff_L  * qL + k_diff_R  * qR + k_diff_0
//        q_hengL*  = d_L       * qL + e_L
//        q_hengR*  = d_R       * qR + e_R
//
//    Dette er det samme som RockerBogieKinematics-noden gjoer for
//    RViz, slik at Gazebo- og RViz-animasjonen er konsistent. I
//    aapen-kjede-URDF-en er disse leddene ikke fysisk koblet til
//    rockerne, saa vi driver dem med moment mot en beregnet
//    maalvinkel.
//
// Alle parametere er SDF-valgfrie (defaults i parentes):
//
//   <left_rocker_joint>   (rocker_left_joint)
//   <right_rocker_joint>  (rocker_right_joint)
//   <diff_joint>          (rocker_bogie_diff_joint)
//   <hengsel_left_joint>  (hengsel_diff_L_joint)
//   <hengsel_right_joint> (hengsel_diff_R_joint)
//
//   <kp>          (200.0)  gain paa (qL + qR)
//   <kd>          ( 20.0)  demping paa (dqL + dqR)
//   <target_sum>  (  0.0)
//   <max_torque>  ( 25.0)  clamp paa all applied torque [Nm]
//
//   <k_diff_L>    ( 0.5)   q_diff* = k_diff_L*qL + k_diff_R*qR + k_diff_0
//   <k_diff_R>    (-0.5)
//   <k_diff_0>    ( 0.0)
//   <d_L>         (-1.0)   q_hengL* = d_L*qL + e_L
//   <e_L>         ( 0.0)
//   <d_R>         (-1.0)   q_hengR* = d_R*qR + e_R
//   <e_R>         ( 0.0)
//
//   <kp_aux>      (40.0)   PD-gain paa kosmetiske ledd
//   <kd_aux>      ( 4.0)
//   <max_torque_aux> ( 5.0)
//
//   <enable>      (true)

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace moonmapper
{

class RockerBogieDifferential
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  // ------------------------------------------------------------------
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[RockerBogieDifferential] Plugin must be attached to a <model>\n";
      return;
    }

    // Joint-navn
    this->left_joint_name_    = _sdf->Get<std::string>(
      "left_rocker_joint",   std::string{"rocker_left_joint"}).first;
    this->right_joint_name_   = _sdf->Get<std::string>(
      "right_rocker_joint",  std::string{"rocker_right_joint"}).first;
    this->diff_joint_name_    = _sdf->Get<std::string>(
      "diff_joint",          std::string{"rocker_bogie_diff_joint"}).first;
    

    // Primaer-PD (sum-constraint på rockers)
    this->kp_         = _sdf->Get<double>("kp",         200.0).first;
    this->kd_         = _sdf->Get<double>("kd",          20.0).first;
    this->target_sum_ = _sdf->Get<double>("target_sum",   0.0).first;
    this->max_torque_ = _sdf->Get<double>("max_torque",  25.0).first;

    // Kinematikk-koeffisienter for de tre kosmetiske ledd
    this->k_diff_L_ = _sdf->Get<double>("k_diff_L",  0.5).first;
    this->k_diff_R_ = _sdf->Get<double>("k_diff_R", -0.5).first;
    this->k_diff_0_ = _sdf->Get<double>("k_diff_0",  0.0).first;
    this->d_L_      = _sdf->Get<double>("d_L",      -1.0).first;
    this->e_L_      = _sdf->Get<double>("e_L",       0.0).first;
    this->d_R_      = _sdf->Get<double>("d_R",      -1.0).first;
    this->e_R_      = _sdf->Get<double>("e_R",       0.0).first;

    // Sekundaer-PD for de kosmetiske ledd (mykere så de ikke
    // forstyrrer base-dynamikken).
    this->kp_aux_        = _sdf->Get<double>("kp_aux",         40.0).first;
    this->kd_aux_        = _sdf->Get<double>("kd_aux",          4.0).first;
    this->max_torque_aux_= _sdf->Get<double>("max_torque_aux",  5.0).first;

    this->enable_     = _sdf->Get<bool>("enable",        true).first;

    gzmsg << "[RockerBogieDifferential] Configured for model '"
          << this->model_.Name(_ecm) << "':\n"
          << "  left_rocker_joint  = " << this->left_joint_name_    << "\n"
          << "  right_rocker_joint = " << this->right_joint_name_   << "\n"
          << "  diff_joint         = " << this->diff_joint_name_    << "\n"
          << "  sum-PD:  kp=" << this->kp_ << " kd=" << this->kd_
          << " target_sum=" << this->target_sum_
          << " max_tau=" << this->max_torque_ << "\n"
          << "  aux-PD:  kp=" << this->kp_aux_ << " kd=" << this->kd_aux_
          << " max_tau=" << this->max_torque_aux_ << "\n"
          << "  q_diff*  = " << this->k_diff_L_ << "*qL + "
                             << this->k_diff_R_ << "*qR + "
                             << this->k_diff_0_ << "\n";
  }

  // ------------------------------------------------------------------
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (!this->enable_ || _info.paused) {
      return;
    }

    // Søk opp alle joint-entiteter første gang de er tilgjengelige.
    CacheJoint(_ecm, this->left_joint_name_,    this->left_joint_);
    CacheJoint(_ecm, this->right_joint_name_,   this->right_joint_);
    CacheJoint(_ecm, this->diff_joint_name_,    this->diff_joint_);
    CacheJoint(_ecm, this->hengsel_l_name_,     this->hengsel_l_joint_);
    CacheJoint(_ecm, this->hengsel_r_name_,     this->hengsel_r_joint_);

    if (this->left_joint_ == gz::sim::kNullEntity ||
        this->right_joint_ == gz::sim::kNullEntity)
    {
      if (!this->warned_missing_) {
        gzwarn << "[RockerBogieDifferential] Fant ikke rocker-joints '"
               << this->left_joint_name_ << "' eller '"
               << this->right_joint_name_ << "' enda.\n";
        this->warned_missing_ = true;
      }
      return;
    }

    // Sikre at nodens komponenter finnes på ALLE relevante joints.
    EnsureJointComponents(_ecm, this->left_joint_);
    EnsureJointComponents(_ecm, this->right_joint_);
    if (this->diff_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->diff_joint_);
    }
    if (this->hengsel_l_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->hengsel_l_joint_);
    }
    if (this->hengsel_r_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->hengsel_r_joint_);
    }

    // ---- Les rocker-tilstand -------------------------------------
    double qL = 0.0, qR = 0.0, dqL = 0.0, dqR = 0.0;
    if (!ReadJointState(_ecm, this->left_joint_,  qL, dqL) ||
        !ReadJointState(_ecm, this->right_joint_, qR, dqR))
    {
      return;
    }

    // ---- 1) Sum-constraint på rockers --------------------------------
    {
      const double err     = (qL + qR) - this->target_sum_;
      const double err_dot = (dqL + dqR);
      double tau = -this->kp_ * err - this->kd_ * err_dot;
      tau = std::clamp(tau, -this->max_torque_, this->max_torque_);
      _ecm.SetComponentData<gz::sim::components::JointForceCmd>(
        this->left_joint_,  {tau});
      _ecm.SetComponentData<gz::sim::components::JointForceCmd>(
        this->right_joint_, {tau});
    }

    // ---- 2) PD-posisjonskontroll på kosmetiske ledd ---------------
    if (this->diff_joint_ != gz::sim::kNullEntity) {
      const double target = this->k_diff_L_ * qL
                          + this->k_diff_R_ * qR
                          + this->k_diff_0_;
      DrivePositionPD(_ecm, this->diff_joint_, target);
    }
    if (this->hengsel_l_joint_ != gz::sim::kNullEntity) {
      const double target = this->d_L_ * qL + this->e_L_;
      DrivePositionPD(_ecm, this->hengsel_l_joint_, target);
    }
    if (this->hengsel_r_joint_ != gz::sim::kNullEntity) {
      const double target = this->d_R_ * qR + this->e_R_;
      DrivePositionPD(_ecm, this->hengsel_r_joint_, target);
    }
  }

private:
  // ------------------------------------------------------------------
  template<typename T>
  static void EnsureComponent(
    gz::sim::EntityComponentManager & _ecm,
    const gz::sim::Entity & _entity)
  {
    if (!_ecm.Component<T>(_entity)) {
      _ecm.CreateComponent(_entity, T());
    }
  }

  static void EnsureJointComponents(
    gz::sim::EntityComponentManager & _ecm,
    const gz::sim::Entity & _entity)
  {
    EnsureComponent<gz::sim::components::JointPosition>(_ecm, _entity);
    EnsureComponent<gz::sim::components::JointVelocity>(_ecm, _entity);
  }

  void CacheJoint(
    gz::sim::EntityComponentManager & _ecm,
    const std::string & _name,
    gz::sim::Entity & _entity) const
  {
    if (_entity == gz::sim::kNullEntity && !_name.empty()) {
      _entity = this->model_.JointByName(_ecm, _name);
    }
  }

  static bool ReadJointState(
    const gz::sim::EntityComponentManager & _ecm,
    const gz::sim::Entity & _entity,
    double & _q, double & _dq)
  {
    const auto * pos =
      _ecm.Component<gz::sim::components::JointPosition>(_entity);
    const auto * vel =
      _ecm.Component<gz::sim::components::JointVelocity>(_entity);
    if (!pos || !vel || pos->Data().empty() || vel->Data().empty()) {
      return false;
    }
    _q  = pos->Data()[0];
    _dq = vel->Data()[0];
    return true;
  }

  // PD-kontroll på en enkelt joint mot en målvinkel.
  void DrivePositionPD(
    gz::sim::EntityComponentManager & _ecm,
    const gz::sim::Entity & _entity,
    double _target) const
  {
    double q = 0.0, dq = 0.0;
    if (!ReadJointState(_ecm, _entity, q, dq)) {
      return;
    }
    double tau = -this->kp_aux_ * (q - _target) - this->kd_aux_ * dq;
    tau = std::clamp(tau, -this->max_torque_aux_, this->max_torque_aux_);
    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(_entity, {tau});
  }

  // ------------------------------------------------------------------
  gz::sim::Model  model_{gz::sim::kNullEntity};

  // Joint-navn og -entiteter
  std::string     left_joint_name_;
  std::string     right_joint_name_;
  std::string     diff_joint_name_;
  std::string     hengsel_l_name_;
  std::string     hengsel_r_name_;

  gz::sim::Entity left_joint_{gz::sim::kNullEntity};
  gz::sim::Entity right_joint_{gz::sim::kNullEntity};
  gz::sim::Entity diff_joint_{gz::sim::kNullEntity};
  gz::sim::Entity hengsel_l_joint_{gz::sim::kNullEntity};
  gz::sim::Entity hengsel_r_joint_{gz::sim::kNullEntity};

  // Primaer-PD på rocker-summen
  double kp_{200.0};
  double kd_{20.0};
  double target_sum_{0.0};
  double max_torque_{25.0};

  // Kinematikk-koeffisienter
  double k_diff_L_{ 0.5};
  double k_diff_R_{-0.5};
  double k_diff_0_{ 0.0};
  double d_L_{-1.0};
  double e_L_{ 0.0};
  double d_R_{-1.0};
  double e_R_{ 0.0};

    // Sekundaer-PD på kosmetiske ledd
  double kp_aux_{40.0};
  double kd_aux_{4.0};
  double max_torque_aux_{5.0};

  bool   enable_{true};
  bool   warned_missing_{false};
};

}  // namespace moonmapper

GZ_ADD_PLUGIN(
  moonmapper::RockerBogieDifferential,
  gz::sim::System,
  moonmapper::RockerBogieDifferential::ISystemConfigure,
  moonmapper::RockerBogieDifferential::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  moonmapper::RockerBogieDifferential,
  "moonmapper::RockerBogieDifferential")
