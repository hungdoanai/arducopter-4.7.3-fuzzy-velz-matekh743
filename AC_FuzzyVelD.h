#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Fuzzy-PID gain scheduling controller for the vertical (D/Z-axis) velocity loop.
// Zero-order Takagi-Sugeno 7x7 centrally symmetric Fuzzy-PID for ArduCopter AC_PosControl (Copter-4.7.3).
// Exposes Mission Planner parameters under PSC_D_FZ_* (and Q_P_D_FZ_* on QuadPlane),
// including optional RC transmitter switch toggle via PSC_D_FZ_RC.
class AC_FuzzyVelD {
public:
    AC_FuzzyVelD();

    CLASS_NO_COPY(AC_FuzzyVelD);

    // Returns true if fuzzy gain scheduling is currently active
    // (controlled by PSC_D_FZ_RC TX switch if > 0, otherwise PSC_D_FZ_ENABLE).
    // Automatically resets internal state on OFF transition and notifies GCS/OSD.
    bool enabled();

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

    // Evaluates 7 overlapping triangular membership functions over [-1, 1] (partition of unity).
    static void fuzzify(float x_norm, float mu[NUM_MF]);

    // Zero-order Sugeno inference (product T-norm + weighted average defuzzification on [-1, 1]).
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
    AP_Int8  _rc_chan;      // Optional RC channel (5..16) to toggle Fuzzy via TX switch (0 = disabled)

    // Internal runtime state (no dynamic allocation)
    bool  _reset_filter;
    bool  _rc_active;
    bool  _last_active;
    float _prev_error;
    float _deriv_filt;
    float _delta_i_term;
    float _last_delta_kp;
    float _last_delta_ki;
    float _last_delta_kd;
};
