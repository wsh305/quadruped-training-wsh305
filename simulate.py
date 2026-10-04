import mujoco
import mujoco.viewer
import time
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
for i in range(model.nu):
    model.actuator_gainprm[i, 0] = 0.0
    model.actuator_biasprm[i, 1] = 0.0
    model.actuator_biasprm[i, 2] = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)