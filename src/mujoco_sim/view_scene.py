import os
import time
import mujoco
import mujoco.viewer

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(script_dir, "models", "scene.xml")
    
    print(f"Loading MuJoCo scene: {scene_path}")
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    
    print("Launching passive viewer window...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close viewer on 'ESC' or window close button
        while viewer.is_running():
            step_start = time.time()
            
            # Step simulation physics
            mujoco.mj_step(model, data)
            
            # Synchronize viewer state
            viewer.sync()
            
            # Maintain real-time simulation pace
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
