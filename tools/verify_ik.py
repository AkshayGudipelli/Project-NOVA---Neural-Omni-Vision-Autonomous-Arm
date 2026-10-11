"""Verify nova_ik.hpp against the real MJCF (kinematics + gravity physics). No ROS needed.
Run from your repo root:  g++ -std=c++17 -I src/controller/include src/../tools/ik_cli.cpp -o tools/ik_cli
                          python3 tools/verify_ik.py src/mujoco_sim/models/scene.xml
"""
import sys, subprocess, numpy as np, mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def grasp_pt():
    l5 = bid("link5"); return d.xpos[l5] + d.xmat[l5].reshape(3,3) @ np.array([0,0,0.055]) - d.xpos[bid("base_link")]
def ik(t):
    o = subprocess.run(["tools/ik_cli"], input=f"{t[0]} {t[1]} {t[2]}\n", capture_output=True, text=True).stdout.split()
    return None if o[0] != "OK" else np.array(list(map(float, o[1:6])))
bad = 0
for t in [(0,.20,.02),(0,.27,.02),(-.16,.17,.02),(.16,.17,.02),(-.20,.10,.02),(.20,.10,.02),(0,.15,.02),(0,.20,.10)]:
    q = ik(t)
    if q is None: print(t, "rejected"); continue
    mujoco.mj_resetData(m, d); d.qpos[:5] = q; mujoco.mj_forward(m, d); ek = np.linalg.norm(grasp_pt()-t)*1000
    mujoco.mj_resetData(m, d); d.ctrl[:5] = q; d.ctrl[5:7] = 0.02
    for _ in range(2500): mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d); ep = np.linalg.norm(grasp_pt()-t)*1000
    flag = "PASS" if ek < 2 and ep < 10 else "FAIL"; bad += flag == "FAIL"
    print(f"{t}: kinematic {ek:5.2f} mm | physics {ep:5.1f} mm  {flag}")
sys.exit(bad)
