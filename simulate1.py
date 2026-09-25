import mujoco
import mujoco.viewer
import time
import threading

class RobotSimulation:
    def __init__(self, xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.running = True
        self.data.ctrl[:] = 0.0
        self.lock = threading.Lock()

    def physics_loop(self):
        while self.running:
            with self.lock:
                self.data.ctrl[:] = 0.0
                mujoco.mj_step(self.model, self.data)
            time.sleep(self.model.opt.timestep * 0.5)

    def render_loop(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            physics_thread = threading.Thread(target=self.physics_loop)
            physics_thread.start()

            while viewer.is_running():
                step_start = time.time()
                with self.lock:
                    viewer.sync()
                
                time_until_next_frame = 0.016 - (time.time() - step_start)
                if time_until_next_frame > 0:
                    time.sleep(time_until_next_frame)
            
            self.running = False
            physics_thread.join()

if __name__ == "__main__":
    sim = RobotSimulation("机器狗.xml")
    sim.render_loop()