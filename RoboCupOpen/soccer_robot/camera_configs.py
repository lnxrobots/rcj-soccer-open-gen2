import soccer_robot.utils.camera as cam

# Camera configurations
ARDUCAM_B0310 = cam.CameraConfig(
    size=(1152, 648),
    raw_size=(2304, 1296),
    fps=30,
    hfov=120,
    f_len_mm=2.87,
    diag_mm=7.4
)

ARDUCAM_B0262 = cam.CameraConfig(
    size=(1014, 760),
    raw_size=(2028, 1520),
    fps=30,
    hfov=75,
    f_len_mm=3.9,
    diag_mm=7.9
)

# Mount configurations
MIRROR_MOUNT = cam.MountConfig(position=(0, 0, 120), pitch=-90, roll=180)
GEN2_FRONT_MOUNT = cam.MountConfig(position=(0, 45, 105), pitch=30, roll=0)
# GEN1_FRONT_MOUNT = cam.MountConfig(position=(0, 45, 105), pitch=30, roll=0)
# GEN1_FRONT_BOTTOM_MOUNT = cam.MountConfig(position=(0, -45, 105), pitch=-30, roll=0)

# Mirror configurations
VINDIS_VACUUM_MIRROR = cam.MirrorConfig(
    field_radius=1700,
    correction_coefficients=(0.0018676073, -0.2821336212, 169.2039182379)
)
