import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

for i in range(model.nu):
    model.actuator_gainprm[i, 0] = 0.0
    model.actuator_biasprm[i, 1] = 0.0
    model.actuator_biasprm[i, 2] = 0.0

MODE_DAMPING = 0
MODE_STANDING = 1
current_mode = MODE_DAMPING

target_stand_angles = np.array([
    0.0, 0.4, -1.5,
    0.0, -0.4, 1.5,
    0.0, -0.4, 1.5,
    0.0, 0.4, -1.5,
])

Kp = 200.0
Kd = 12.0

stand_start_time = 0.0
stand_duration = 1.0
start_angles = np.zeros(12)
is_standing_moving = False

damping_kd = 10.0

def key_callback(keycode):
    global current_mode, is_standing_moving, start_angles, stand_start_time
    if keycode == 49:
        current_mode = MODE_DAMPING
        is_standing_moving = False
        for i in range(model.nu):
            model.actuator_gainprm[i, 0] = 0.0
            model.actuator_biasprm[i, 1] = 0.0
            model.actuator_biasprm[i, 2] = 0.0
        print("阻尼模式")
    elif keycode == 50:
        current_mode = MODE_STANDING
        start_angles = data.qpos[7:19].copy()
        stand_start_time = data.time
        is_standing_moving = True
        for i in range(model.nu):
            model.actuator_gainprm[i, 0] = 1.0
            model.actuator_biasprm[i, 1] = 0.0
            model.actuator_biasprm[i, 2] = 0.0
        print("站立模式，start_angles =", start_angles)

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.elevation = -20

    while viewer.is_running():
        step_start = time.time()

        if current_mode == MODE_DAMPING:
            data.ctrl[:] = -damping_kd * data.qvel[6:18]

        elif current_mode == MODE_STANDING:
            if is_standing_moving:
                elapsed = data.time - stand_start_time
                if elapsed >= stand_duration:
                    is_standing_moving = False
                    print("站立完成")
                    calc_target = target_stand_angles
                else:
                    alpha = elapsed / stand_duration
                    calc_target = start_angles + alpha * (target_stand_angles - start_angles)
            else:
                calc_target = target_stand_angles

            torque = Kp * (calc_target - data.qpos[7:19]) - Kd * data.qvel[6:18]
            data.ctrl[:] = np.clip(torque, -20.0, 20.0)

            data.qpos[0] = 0.0
            data.qpos[1] = 0.0
            data.qvel[0] = 0.0
            data.qvel[1] = 0.0

        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)