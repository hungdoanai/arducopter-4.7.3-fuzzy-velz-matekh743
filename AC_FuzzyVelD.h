#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Fuzzy-PID gain scheduling controller for the vertical (D/Z-axis) velocity loop.
// Adapted from Mamdani 7x7 fuzzy velocity PID tuning for ArduCopter AC_PosControl.
// Exposes Mission Planner parameters under PSC_D_FZ_* (and Q_P_D_FZ_* on QuadPlane).
class AC_FuzzyVelD {
public:
    AC_FuzzyVelD();

    CLASS_NO_COPY(AC_FuzzyVelD);

    // Returns true if fuzzy gain scheduling is enabled (PSC_D_FZ_ENABLE == 1).
    bool enabled() const { return _enable.get() != 0; }

    // Resets internal derivative filter and fuzzy delta-I integrator state.
    void reset();

    // Computes the additional fuzzy-scheduled PID acceleration correction (m/s^2)
    // to be summed with the base AC_PID_Basic output:
    //   u_total = u_base_pid + delta_kp * e + integral(delta_ki * e dt) + delta_kd * de/dt
    // Thread-safe: never mutates AP_Float parameters of _pid_vel_d_m.
    float update(float vel_error_ms,
                 float dt_s,
                 bool limit_neg,
                 bool limit_pos,
                 float base_kp,
                 float base_ki,
                 float base_kd,
                 float imax_mss);

    // Accessors for diagnostics / testing
    float get_delta_kp() const { return _last_delta_kp; }
    float get_delta_ki() const { return _last_delta_ki; }
    float get_delta_kd() const { return _last_delta_kd; }
    float get_integrator() const { return _delta_i_term; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // 7 linguistic terms: NB=-3, NM=-2, NS=-1, ZO=0, PS=+1, PM=+2, PB=+3
    static constexpr uint8_t NUM_MF = 7;

    // Evaluates 7 overlapping triangular membership functions over [-1, 1].
    static void fuzzify(float x_norm, float mu[NUM_MF]);

    // Mamdani min-max inference with centroid defuzzification on [-1, 1].
    static void infer(const float mu_e[NUM_MF],
                      const float mu_ec[NUM_MF],
                      float &out_kp_norm,
                      float &out_ki_norm,
                      float &out_kd_norm);

    // Mission Planner configurable parameters
    AP_Int8  _enable;       // 0: Disabled (stock PID), 1: Enabled (Fuzzy-PID)
    AP_Float _e_max;        // Velocity error normalization range (m/s)
    AP_Float _ed_max;       // Velocity error derivative normalization range (m/s^2)
    AP_Float _dp_max;       // Max |delta Kp| adjustment
    AP_Float _di_max;       // Max |delta Ki| adjustment
    AP_Float _dd_max;       // Max |delta Kd| adjustment
    AP_Float _filt_d_hz;    // Low-pass filter cutoff for error derivative (Hz)

    // Internal runtime state (no dynamic allocation)
    bool  _reset_filter;
    float _prev_error;
    float _deriv_filt;
    float _delta_i_term;
    float _last_delta_kp;
    float _last_delta_ki;
    float _last_delta_kd;
};
