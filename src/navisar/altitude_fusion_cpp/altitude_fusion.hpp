#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace navisar {

struct FusionConfig {
  double lidar_min_m = 0.08;
  double lidar_max_m = 8.0;
  double max_tilt_deg = 20.0;
  double lidar_timeout_s = 0.2;
  double max_agl_rate_mps = 6.0;
  double baro_lpf_alpha = 0.15;
  double lidar_lpf_alpha = 0.35;
  double gps_alt_lpf_alpha = 0.20;
  double ground_update_gamma_strong = 0.05;
  double ground_update_gamma_weak = 0.01;
  double correction_gain_strong = 0.15;
  double correction_gain_weak = 0.03;
  double innovation_max_m = 0.75;
  double ground_jump_strong_m = 0.8;
  double ground_jump_reject_m = 2.0;
  double lidar_mount_roll_rad = 0.0;
  double lidar_mount_pitch_rad = 0.0;
  double lidar_quality_min = 20.0;
  double innovation_reject_m = 3.0;
};

struct MavSample {
  double timestamp_s = 0.0;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;
  double alt_amsl_m = 0.0;
  double relative_alt_m = 0.0;
};

struct LidarSample {
  double timestamp_s = 0.0;
  double distance_m = 0.0;
  std::optional<double> quality;
  bool healthy = true;
};

struct FusionOutput {
  double timestamp_s = 0.0;
  std::string mode;
  double h_amsl_pred_m = 0.0;
  double h_amsl_est_m = 0.0;
  double h_ground_est_m = 0.0;
  std::optional<double> h_agl_filt_m;
  bool lidar_valid = false;
  double lidar_confidence = 0.0;
  double correction_gain = 0.0;
  double ground_gamma = 0.0;
};

class AltitudeFusion {
 public:
  explicit AltitudeFusion(const FusionConfig& cfg = FusionConfig()) : cfg_(cfg) {}

  FusionOutput Step(const MavSample& mav, const std::optional<LidarSample>& lidar);
  double GpsOutputAltitudeM() const { return h_gps_out_m_; }

 private:
  static constexpr const char* kStartup = "startup_ground_lock";
  static constexpr const char* kSameSurface = "same_surface_low_altitude";
  static constexpr const char* kUncertain = "uncertain_terrain_transition";
  static constexpr const char* kLidarInvalid = "lidar_invalid_baro_only";

  static double Clip(double v, double lo, double hi);
  static double Lpf(std::optional<double> prev, double v, double alpha);
  double TiltAgl(double dist_m, double roll_rad, double pitch_rad) const;
  void Initialize(const MavSample& mav, const std::optional<LidarSample>& lidar);

  FusionConfig cfg_;
  bool initialized_ = false;

  double h_amsl_est_m_ = 0.0;
  double h_ground_est_m_ = 0.0;
  double h_gps_out_m_ = 0.0;

  std::optional<double> baro_rel_filt_m_;
  std::optional<double> baro_rel_prev_filt_m_;
  std::optional<double> h_agl_filt_m_;
  std::optional<double> h_agl_prev_m_;
  std::optional<double> last_step_t_s_;
  std::string mode_ = kStartup;
};

}  // namespace navisar
