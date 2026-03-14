#include "altitude_fusion.hpp"

namespace navisar {

double AltitudeFusion::Clip(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

double AltitudeFusion::Lpf(std::optional<double> prev, double v, double alpha) {
  if (!prev.has_value()) return v;
  return alpha * v + (1.0 - alpha) * prev.value();
}

double AltitudeFusion::TiltAgl(double dist_m, double roll_rad, double pitch_rad) const {
  return dist_m * std::cos(roll_rad + cfg_.lidar_mount_roll_rad) *
         std::cos(pitch_rad + cfg_.lidar_mount_pitch_rad);
}

void AltitudeFusion::Initialize(const MavSample& mav,
                                const std::optional<LidarSample>& lidar) {
  h_amsl_est_m_ = mav.alt_amsl_m;
  h_gps_out_m_ = mav.alt_amsl_m;
  baro_rel_filt_m_ = mav.relative_alt_m;
  baro_rel_prev_filt_m_ = mav.relative_alt_m;

  if (lidar.has_value() && lidar->healthy) {
    const double agl = TiltAgl(lidar->distance_m, mav.roll_rad, mav.pitch_rad);
    h_agl_filt_m_ = agl;
    h_agl_prev_m_ = agl;
    h_ground_est_m_ = h_amsl_est_m_ - agl;
  } else {
    h_ground_est_m_ = h_amsl_est_m_;
  }

  mode_ = kStartup;
  last_step_t_s_ = mav.timestamp_s;
  initialized_ = true;
}

FusionOutput AltitudeFusion::Step(const MavSample& mav,
                                  const std::optional<LidarSample>& lidar) {
  if (!initialized_) {
    Initialize(mav, lidar);
  }

  baro_rel_filt_m_ = Lpf(baro_rel_filt_m_, mav.relative_alt_m, cfg_.baro_lpf_alpha);
  double delta_baro_m = 0.0;
  if (baro_rel_prev_filt_m_.has_value()) {
    delta_baro_m = baro_rel_filt_m_.value() - baro_rel_prev_filt_m_.value();
  }
  const double h_amsl_pred_m = h_amsl_est_m_ + delta_baro_m;

  bool lidar_valid = false;
  double lidar_conf = 0.0;
  double correction_gain = 0.0;
  double ground_gamma = 0.0;

  if (lidar.has_value()) {
    const double roll_deg = std::abs(mav.roll_rad * 180.0 / M_PI);
    const double pitch_deg = std::abs(mav.pitch_rad * 180.0 / M_PI);
    const bool range_ok = lidar->distance_m > cfg_.lidar_min_m && lidar->distance_m < cfg_.lidar_max_m;
    const bool tilt_ok = roll_deg <= cfg_.max_tilt_deg && pitch_deg <= cfg_.max_tilt_deg;
    const bool fresh_ok = (mav.timestamp_s - lidar->timestamp_s) <= cfg_.lidar_timeout_s;
    bool quality_ok = true;
    double quality_factor = 1.0;
    if (lidar->quality.has_value()) {
      quality_ok = lidar->quality.value() >= cfg_.lidar_quality_min;
      quality_factor = Clip(lidar->quality.value() / 100.0, 0.0, 1.0);
    }

    const double h_agl_raw_m = TiltAgl(lidar->distance_m, mav.roll_rad, mav.pitch_rad);
    bool agl_rate_ok = true;
    if (h_agl_prev_m_.has_value() && last_step_t_s_.has_value()) {
      const double dt = std::max(1e-3, mav.timestamp_s - last_step_t_s_.value());
      const double agl_rate = std::abs((h_agl_raw_m - h_agl_prev_m_.value()) / dt);
      agl_rate_ok = agl_rate <= cfg_.max_agl_rate_mps;
    }

    const double h_amsl_lidar = h_ground_est_m_ + h_agl_raw_m;
    const double innovation_abs = std::abs(h_amsl_lidar - h_amsl_pred_m);
    const bool innovation_ok = innovation_abs <= cfg_.innovation_reject_m;

    lidar_valid = lidar->healthy && range_ok && tilt_ok && fresh_ok && quality_ok && agl_rate_ok && innovation_ok;
    if (lidar_valid) {
      const double tilt_factor = Clip(1.0 - std::max(roll_deg, pitch_deg) / cfg_.max_tilt_deg, 0.0, 1.0);
      lidar_conf = quality_factor * tilt_factor;

      h_agl_filt_m_ = Lpf(h_agl_filt_m_, h_agl_raw_m, cfg_.lidar_lpf_alpha);
      const double h_ground_meas = h_amsl_pred_m - h_agl_filt_m_.value();
      const double e_ground = h_ground_meas - h_ground_est_m_;
      const double jump_abs = std::abs(e_ground);

      double base_gain = 0.0;
      double base_gamma = 0.0;
      if (jump_abs < cfg_.ground_jump_strong_m) {
        base_gain = cfg_.correction_gain_strong;
        base_gamma = cfg_.ground_update_gamma_strong;
        mode_ = kSameSurface;
      } else if (jump_abs < cfg_.ground_jump_reject_m) {
        base_gain = cfg_.correction_gain_weak;
        base_gamma = cfg_.ground_update_gamma_weak;
        mode_ = kUncertain;
      } else {
        mode_ = kUncertain;
      }

      correction_gain = base_gain * lidar_conf;
      ground_gamma = base_gamma * lidar_conf;

      if (ground_gamma > 0.0) {
        h_ground_est_m_ += ground_gamma * e_ground;
      }

      const double h_amsl_lidar_corr = h_ground_est_m_ + h_agl_filt_m_.value();
      const double innovation = h_amsl_lidar_corr - h_amsl_pred_m;
      const double innovation_clamped = Clip(innovation, -cfg_.innovation_max_m, cfg_.innovation_max_m);
      h_amsl_est_m_ = h_amsl_pred_m + correction_gain * innovation_clamped;
      h_agl_prev_m_ = h_agl_raw_m;
    }
  }

  if (!lidar_valid) {
    mode_ = kLidarInvalid;
    h_amsl_est_m_ = h_amsl_pred_m;
  }

  h_gps_out_m_ = Lpf(h_gps_out_m_, h_amsl_est_m_, cfg_.gps_alt_lpf_alpha);
  baro_rel_prev_filt_m_ = baro_rel_filt_m_;
  last_step_t_s_ = mav.timestamp_s;

  FusionOutput out;
  out.timestamp_s = mav.timestamp_s;
  out.mode = mode_;
  out.h_amsl_pred_m = h_amsl_pred_m;
  out.h_amsl_est_m = h_amsl_est_m_;
  out.h_ground_est_m = h_ground_est_m_;
  out.h_agl_filt_m = h_agl_filt_m_;
  out.lidar_valid = lidar_valid;
  out.lidar_confidence = lidar_conf;
  out.correction_gain = correction_gain;
  out.ground_gamma = ground_gamma;
  return out;
}

}  // namespace navisar
