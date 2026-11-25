#include "AC_FuzzyVelD.h"

#include <cmath>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>

// Default parameter values tuned for ArduCopter 4.7.3 SI-unit (m/s, m/s^2) Z-velocity loop
// Default PSC_D_VEL_P = 5.0, PSC_D_VEL_I = 0.0, PSC_D_VEL_D = 0.0
#define FUZZY_VELD_ENABLE_DEFAULT   0
#define FUZZY_VELD_E_MAX_DEFAULT    1.5f    // m/s
#define FUZZY_VELD_ED_MAX_DEFAULT   3.0f    // m/s^2
#define FUZZY_VELD_DP_MAX_DEFAULT   2.0f    // max +/- delta Kp
#define FUZZY_VELD_DI_MAX_DEFAULT   0.5f    // max +/- delta Ki
#define FUZZY_VELD_DD_MAX_DEFAULT   0.10f   // max +/- delta Kd
#define FUZZY_VELD_FILT_D_DEFAULT   5.0f    // Hz
#define FUZZY_VELD_RC_CHAN_DEFAULT  0       // 0 = use PSC_D_FZ_ENABLE only; 5..16 = TX RC channel

#define FUZZY_VELD_RC_PWM_ON        1700
#define FUZZY_VELD_RC_PWM_OFF       1300
#define FUZZY_VELD_RC_PWM_MIN_VALID 900
#define FUZZY_VELD_RC_PWM_MAX_VALID 2100

const AP_Param::GroupInfo AC_FuzzyVelD::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Vertical velocity Fuzzy-PID enable
    // @Description: Enables symmetric zero-order Sugeno Fuzzy gain-scheduling PID in the vertical (Z/Down) velocity loop when PSC_D_FZ_RC is 0. Set to 0 for standard ArduCopter PID.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AC_FuzzyVelD, _enable, FUZZY_VELD_ENABLE_DEFAULT),

    // @Param: E_MAX
    // @DisplayName: Vertical velocity Fuzzy error scale
    // @Description: Normalization range for vertical velocity error (m/s). Errors beyond +/-E_MAX saturate to +/-1.0 in the fuzzy universe.
    // @Units: m/s
    // @Range: 0.2 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("E_MAX", 2, AC_FuzzyVelD, _e_max, FUZZY_VELD_E_MAX_DEFAULT),

    // @Param: ED_MAX
    // @DisplayName: Vertical velocity Fuzzy error rate scale
    // @Description: Normalization range for vertical velocity error derivative (m/s/s).
    // @Units: m/s/s
    // @Range: 0.5 15.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("ED_MAX", 3, AC_FuzzyVelD, _ed_max, FUZZY_VELD_ED_MAX_DEFAULT),

    // @Param: DP_MAX
    // @DisplayName: Vertical velocity Fuzzy max delta P gain
    // @Description: Maximum +/- proportional gain adjustment added to PSC_D_VEL_P by the zero-order Sugeno fuzzy inference engine.
    // @Range: 0.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DP_MAX", 4, AC_FuzzyVelD, _dp_max, FUZZY_VELD_DP_MAX_DEFAULT),

    // @Param: DI_MAX
    // @DisplayName: Vertical velocity Fuzzy max delta I gain
    // @Description: Maximum +/- integral gain adjustment added to PSC_D_VEL_I by the zero-order Sugeno fuzzy inference engine.
    // @Range: 0.0 3.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("DI_MAX", 5, AC_FuzzyVelD, _di_max, FUZZY_VELD_DI_MAX_DEFAULT),

    // @Param: DD_MAX
    // @DisplayName: Vertical velocity Fuzzy max delta D gain
    // @Description: Maximum +/- derivative gain adjustment added to PSC_D_VEL_D by the zero-order Sugeno fuzzy inference engine.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DD_MAX", 6, AC_FuzzyVelD, _dd_max, FUZZY_VELD_DD_MAX_DEFAULT),

    // @Param: FILT
    // @DisplayName: Vertical velocity Fuzzy derivative filter
    // @Description: Low-pass filter cutoff frequency (Hz) applied to the vertical velocity error derivative before fuzzification.
    // @Units: Hz
    // @Range: 0.5 30.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("FILT", 7, AC_FuzzyVelD, _filt_d_hz, FUZZY_VELD_FILT_D_DEFAULT),

    // @Param: RC
    // @DisplayName: Vertical velocity Fuzzy TX RC switch channel
    // @Description: Optional RC input channel (5 to 16) to toggle Fuzzy-PID on/off from transmitter switch (PWM >= 1700 ON, <= 1300 OFF). Set to 0 to use PSC_D_FZ_ENABLE parameter only.
    // @Range: 0 16
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RC", 8, AC_FuzzyVelD, _rc_chan, FUZZY_VELD_RC_CHAN_DEFAULT),

    AP_GROUPEND
};

// Input MF centers and zero-order Sugeno output singletons on [-1, 1] for {NB, NM, NS, ZO, PS, PM, PB}
static constexpr float MF_CENTERS[7] = {
    -1.0f, -0.6666667f, -0.3333333f, 0.0f, 0.3333333f, 0.6666667f, 1.0f
};

// Centrally symmetric 7x7 zero-order Sugeno Fuzzy-PID rule tables (rows = e, cols = de/dt)
// Symmetric for both climb and descent: RULE[6-i][6-j] == RULE[i][j]
static constexpr uint8_t RULE_KP[7][7] = {
    {6, 6, 5, 5, 4, 3, 3},
    {6, 5, 5, 4, 4, 3, 2},
    {5, 5, 4, 4, 3, 2, 2},
    {5, 4, 3, 3, 3, 4, 5},
    {2, 2, 3, 4, 4, 5, 5},
    {2, 3, 4, 4, 5, 5, 6},
    {3, 3, 4, 5, 5, 6, 6},
};

static constexpr uint8_t RULE_KI[7][7] = {
    {0, 0, 1, 1, 2, 3, 3},
    {0, 1, 1, 2, 2, 3, 3},
    {1, 2, 3, 4, 4, 3, 2},
    {2, 3, 4, 5, 4, 3, 2},
    {2, 3, 4, 4, 3, 2, 1},
    {3, 3, 2, 2, 1, 1, 0},
    {3, 3, 2, 1, 1, 0, 0},
};

static constexpr uint8_t RULE_KD[7][7] = {
    {6, 5, 4, 3, 2, 1, 2},
    {5, 5, 4, 3, 2, 3, 4},
    {5, 4, 3, 3, 4, 5, 5},
    {6, 5, 4, 3, 4, 5, 6},
    {5, 5, 4, 3, 3, 4, 5},
    {4, 3, 2, 3, 4, 5, 5},
    {2, 1, 2, 3, 4, 5, 6},
};

AC_FuzzyVelD::AC_FuzzyVelD()
    : _reset_filter(true)
    , _rc_active(false)
    , _last_active(false)
    , _prev_error(0.0f)
    , _deriv_filt(0.0f)
    , _delta_i_term(0.0f)
    , _last_delta_kp(0.0f)
    , _last_delta_ki(0.0f)
    , _last_delta_kd(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AC_FuzzyVelD::enabled()
{
    bool active = (_enable.get() != 0);

    const int8_t ch = _rc_chan.get();
    if (ch >= 5 && ch <= 16) {
        const RC_Channel *chan = rc().channel(uint8_t(ch - 1));
        if (chan != nullptr) {
            const int16_t pwm = chan->get_radio_in();
            if (pwm >= FUZZY_VELD_RC_PWM_MIN_VALID && pwm <= FUZZY_VELD_RC_PWM_MAX_VALID) {
                if (pwm >= FUZZY_VELD_RC_PWM_ON) {
                    _rc_active = true;
                } else if (pwm <= FUZZY_VELD_RC_PWM_OFF) {
                    _rc_active = false;
                }
                active = _rc_active;
            } else {
                _rc_active = false;
                active = false;
            }
        }
    } else {
        _rc_active = false;
    }

    if (active != _last_active) {
        _last_active = active;
        if (!active) {
            reset();
        }
#if HAL_GCS_ENABLED
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Fuzzy VelZ: %s", active ? "ON" : "OFF");
#endif
    }

    return active;
}

void AC_FuzzyVelD::reset()
{
    _reset_filter = true;
    _prev_error = 0.0f;
    _deriv_filt = 0.0f;
    _delta_i_term = 0.0f;
    _last_delta_kp = 0.0f;
    _last_delta_ki = 0.0f;
    _last_delta_kd = 0.0f;
}

void AC_FuzzyVelD::fuzzify(float x_norm, float mu[NUM_MF])
{
    const float x = constrain_float(x_norm, -1.0f, 1.0f);
    constexpr float step = 1.0f / 3.0f;

    for (uint8_t i = 0; i < NUM_MF; i++) {
        const float dist = fabsf(x - MF_CENTERS[i]);
        if (dist < step) {
            mu[i] = 1.0f - (dist / step);
        } else {
            mu[i] = 0.0f;
        }
    }
}

void AC_FuzzyVelD::infer(const float mu_e[NUM_MF],
                         const float mu_ec[NUM_MF],
                         float &out_kp_norm,
                         float &out_ki_norm,
                         float &out_kd_norm)
{
    // Zero-order Takagi-Sugeno inference:
    //   w_ij = mu_e[i] * mu_ec[j]   (algebraic product T-norm)
    //   out  = sum(w_ij * z_ij) / sum(w_ij)   (weighted average of singleton consequents)
    float num_kp = 0.0f;
    float num_ki = 0.0f;
    float num_kd = 0.0f;
    float sum_w = 0.0f;

    for (uint8_t i = 0; i < NUM_MF; i++) {
        if (mu_e[i] <= 0.0f) {
            continue;
        }
        for (uint8_t j = 0; j < NUM_MF; j++) {
            if (mu_ec[j] <= 0.0f) {
                continue;
            }
            const float w = mu_e[i] * mu_ec[j];
            sum_w += w;
            num_kp += w * MF_CENTERS[RULE_KP[i][j]];
            num_ki += w * MF_CENTERS[RULE_KI[i][j]];
            num_kd += w * MF_CENTERS[RULE_KD[i][j]];
        }
    }

    if (sum_w <= 1e-6f) {
        out_kp_norm = 0.0f;
        out_ki_norm = 0.0f;
        out_kd_norm = 0.0f;
        return;
    }

    const float inv_w = 1.0f / sum_w;
    out_kp_norm = constrain_float(num_kp * inv_w, -1.0f, 1.0f);
    out_ki_norm = constrain_float(num_ki * inv_w, -1.0f, 1.0f);
    out_kd_norm = constrain_float(num_kd * inv_w, -1.0f, 1.0f);
}

float AC_FuzzyVelD::update(float vel_error_ms,
                           float dt_s,
                           bool limit_neg,
                           bool limit_pos,
                           float base_kp,
                           float base_ki,
                           float base_kd,
                           float imax_mss)
{
    if (!is_positive(dt_s)) {
        reset();
        return 0.0f;
    }

    if (_reset_filter) {
        _reset_filter = false;
        _prev_error = vel_error_ms;
        _deriv_filt = 0.0f;
    } else {
        const float raw_deriv = (vel_error_ms - _prev_error) / dt_s;
        _prev_error = vel_error_ms;
        const float filt_hz = MAX(_filt_d_hz.get(), 0.5f);
        const float alpha = calc_lowpass_alpha_dt(dt_s, filt_hz);
        _deriv_filt += alpha * (raw_deriv - _deriv_filt);
    }

    const float e_scale = MAX(_e_max.get(), 0.1f);
    const float ed_scale = MAX(_ed_max.get(), 0.2f);
    const float e_norm = constrain_float(vel_error_ms / e_scale, -1.0f, 1.0f);
    const float ec_norm = constrain_float(_deriv_filt / ed_scale, -1.0f, 1.0f);

    float mu_e[NUM_MF];
    float mu_ec[NUM_MF];
    fuzzify(e_norm, mu_e);
    fuzzify(ec_norm, mu_ec);

    float out_kp_norm = 0.0f;
    float out_ki_norm = 0.0f;
    float out_kd_norm = 0.0f;
    infer(mu_e, mu_ec, out_kp_norm, out_ki_norm, out_kd_norm);

    const float dp_max = MAX(_dp_max.get(), 0.0f);
    const float di_max = MAX(_di_max.get(), 0.0f);
    const float dd_max = MAX(_dd_max.get(), 0.0f);

    float delta_kp = out_kp_norm * dp_max;
    float delta_ki = out_ki_norm * di_max;
    float delta_kd = out_kd_norm * dd_max;

    const float min_delta_kp = -0.5f * MAX(base_kp, 0.0f);
    if (delta_kp < min_delta_kp) {
        delta_kp = min_delta_kp;
    }
    if (base_ki + delta_ki < 0.0f) {
        delta_ki = -base_ki;
    }
    if (base_kd + delta_kd < 0.0f) {
        delta_kd = -base_kd;
    }

    _last_delta_kp = delta_kp;
    _last_delta_ki = delta_ki;
    _last_delta_kd = delta_kd;

    const float di_step = delta_ki * vel_error_ms * dt_s;
    const bool allow_integrate =
        (!limit_pos || di_step < 0.0f) &&
        (!limit_neg || di_step > 0.0f);
    if (allow_integrate) {
        const float i_lim = MAX(imax_mss * 0.5f, 0.0f);
        _delta_i_term = constrain_float(_delta_i_term + di_step, -i_lim, i_lim);
    }

    return (delta_kp * vel_error_ms) + _delta_i_term + (delta_kd * _deriv_filt);
}
