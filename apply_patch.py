#!/usr/bin/env python3
"""
Apply Fuzzy-PID Z-velocity loop integration into ArduPilot (Copter-4.7.3) AC_PosControl.
Uses regex matching tolerant of whitespace/comment differences between 4.7.x revisions.
"""
import os
import re
import shutil
import sys

def patch_regex(path, patches):
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()

    for desc, check_str, pattern, repl in patches:
        if check_str in content:
            print(f"[SKIP] Already applied: {desc}")
            continue
        new_content, count = re.subn(pattern, repl, content, count=1, flags=re.MULTILINE)
        if count == 0:
            print(f"[ERROR] Pattern not found for: {desc} in {path}", file=sys.stderr)
            sys.exit(1)
        content = new_content
        print(f"[OK] Applied: {desc}")

    with open(path, "w", encoding="utf-8") as f:
        f.write(content)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 apply_patch.py <path_to_ardupilot_repo>")
        sys.exit(1)

    repo_root = os.path.abspath(sys.argv[1])
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ctrl_dir = os.path.join(repo_root, "libraries", "AC_AttitudeControl")

    if not os.path.isdir(ctrl_dir):
        print(f"[ERROR] Not found: {ctrl_dir}", file=sys.stderr)
        sys.exit(1)

    for fname in ("AC_FuzzyVelD.h", "AC_FuzzyVelD.cpp"):
        shutil.copy2(os.path.join(script_dir, fname), os.path.join(ctrl_dir, fname))
        print(f"[OK] Copied {fname} -> {ctrl_dir}")

    pos_h = os.path.join(ctrl_dir, "AC_PosControl.h")
    patch_regex(pos_h, [
        (
            "Include AC_FuzzyVelD.h",
            '#include "AC_FuzzyVelD.h"',
            r'(#include\s+"AC_AttitudeControl\.h"[^\n]*)',
            r'\1\n#include "AC_FuzzyVelD.h"'
        ),
        (
            "Add _fuzzy_vel_d member",
            "_fuzzy_vel_d;",
            r'(AC_PID_Basic\s+_pid_vel_d_m;[^\n]*)',
            r'\1\n    AC_FuzzyVelD    _fuzzy_vel_d;       // Fuzzy-PID gain scheduler for Z/Down velocity loop'
        ),
    ])

    pos_cpp = os.path.join(ctrl_dir, "AC_PosControl.cpp")
    patch_regex(pos_cpp, [
        (
            "Register PSC_D_FZ_* subgroup in var_info[]",
            '"_D_FZ_"',
            r'(AP_SUBGROUPINFO\(_pid_vel_ne_m,\s*"_NE_VEL_",\s*14,\s*AC_PosControl,\s*AC_PID_2D\),)',
            r'\1\n\n'
            r'    // @Group: _D_FZ_\n'
            r'    // @Path: AC_FuzzyVelD.cpp\n'
            r'    AP_SUBGROUPINFO(_fuzzy_vel_d, "_D_FZ_", 15, AC_PosControl, AC_FuzzyVelD),'
        ),
        (
            "Reset _fuzzy_vel_d in D_init_controller()",
            "_fuzzy_vel_d.reset();",
            r'(_pid_vel_d_m\.set_integrator\(0\.0f\);)',
            r'\1\n    _fuzzy_vel_d.reset();'
        ),
        (
            "Apply Fuzzy-PID correction in D_update_controller()",
            "_fuzzy_vel_d.enabled()",
            r'(_accel_target_ned_mss\.z\s*=\s*_pid_vel_d_m\.update_all\([^;]+\);)',
            r'\1\n'
            r'    if (_fuzzy_vel_d.enabled()) {\n'
            r'        const float vel_err_d_ms = _vel_target_ned_ms.z - _vel_estimate_ned_ms.z;\n'
            r'        _accel_target_ned_mss.z += _fuzzy_vel_d.update(\n'
            r'            vel_err_d_ms, _dt_s,\n'
            r'            _motors.limit.throttle_lower, _motors.limit.throttle_upper,\n'
            r'            _pid_vel_d_m.kP(), _pid_vel_d_m.kI(), _pid_vel_d_m.kD(), _pid_vel_d_m.imax());\n'
            r'    }'
        ),
    ])

    print("[SUCCESS] ArduPilot AC_PosControl patched with AC_FuzzyVelD!")

if __name__ == "__main__":
    main()
