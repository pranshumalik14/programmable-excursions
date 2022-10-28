import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import time
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')

here = os.path.dirname(__file__)
xml_path = os.path.join(here, "exo_arm_model.xml")

if not os.path.exists(xml_path):
    raise FileNotFoundError(f"Error: XML file not found at '{xml_path}'")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

init_sho_rad = 45 * np.pi / 180
init_elb_rad = 75 * np.pi / 180

duration = 0.2  # seconds
times = []
sho_ang = []
elb_ang = []
sensor_readings = {
    'sho-left': [],
    'sho-right': [],
    'elb-left': [],
    'elb-right': []
}
sensor_ids = {
    'sho-left': model.sensor('sho_pressure_left').id,
    'sho-right': model.sensor('sho_pressure_right').id,
    'elb-left': model.sensor('elb_pressure_left').id,
    'elb-right': model.sensor('elb_pressure_right').id
}
sho_qposadr = model.joint('human_shoulder').qposadr[0]
elb_qposadr = model.joint('human_elbow').qposadr[0]

print(f"Running interactive simulation for {duration} second...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    # reset start pose
    data.qpos[model.joint('human_shoulder').qposadr] = init_sho_rad
    data.qpos[model.joint('human_elbow').qposadr] = init_elb_rad
    data.qpos[model.joint('exo_shoulder').qposadr] = init_sho_rad
    data.qpos[model.joint('exo_elbow').qposadr] = init_elb_rad
    mujoco.mj_forward(model, data)

    # set the camera view
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -90
    viewer.cam.distance = 1.0
    viewer.cam.lookat[:] = 0.5 * (data.body('human_forearm').xpos +
                                  data.body('human_upper_arm').xpos)
    viewer.cam.distance = 1

    while viewer.is_running() and data.time < duration:
        step_start = time.time()

        # apply constant torque to the exo motors (Nm)
        shoulder_torque = 2
        elbow_torque = 2

        data.ctrl[0] = shoulder_torque
        data.ctrl[1] = elbow_torque

        mujoco.mj_step(model, data)
        times.append(data.time)
        sho_ang.append(data.qpos[sho_qposadr])
        elb_ang.append(data.qpos[elb_qposadr])

        for name, id in sensor_ids.items():
            sensor_readings[name].append(data.sensordata[id])

        viewer.cam.lookat[:] = data.body('human_forearm').xpos
        viewer.sync()

        # slo-mo
        time_until_next_step = (model.opt.timestep * 50) - \
            (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Simulation complete. Plotting results...")

# plot results
fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
fig.suptitle('Cuff Pressure Sensor Readings', fontsize=16)
times_np = np.array(times)
plot_mask = times_np <= 0.15
times_to_plot = times_np[plot_mask]

# shoulder contact forces
axs[0].plot(times_to_plot, np.array(
    sensor_readings['sho-left'])[plot_mask], label="Front")
axs[0].plot(times_to_plot, np.array(
    sensor_readings['sho-right'])[plot_mask], label="Back")
axs[0].set_title('Shoulder Cuff')
axs[0].set_ylabel('Contact Force (N)')
axs[0].legend()
axs[0].grid(True)

# elbow contact forces
axs[1].plot(times_to_plot, np.array(
    sensor_readings['elb-left'])[plot_mask], label="Front")
axs[1].plot(times_to_plot, np.array(
    sensor_readings['elb-right'])[plot_mask], label="Back")
axs[1].set_title('Elbow Cuff')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Contact Force (N)')
axs[1].legend()
axs[1].grid(True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig(os.path.join(here, 'cuff_forces_plot.png'))
print("Plot saved to cuff_forces_plot.png")

# joint deviation plot
sho_dev_deg = (np.array(sho_ang) - init_sho_rad) * 180 / np.pi
elb_dev_deg = (np.array(elb_ang) - init_elb_rad) * 180 / np.pi
fig2, ax2 = plt.subplots(2, 1, figsize=(12, 6), sharex=True, sharey=True)
fig2.suptitle('Arm Deviation from Start Pose', fontsize=16)
ax2[0].plot(times_to_plot, sho_dev_deg[plot_mask], label="Shoulder Deviation")
ax2[0].set_ylabel("Deviation (°)")
ax2[0].set_title("Shoulder")
ax2[0].legend()
ax2[0].grid(True)
ax2[1].plot(times_to_plot, elb_dev_deg[plot_mask], label="Elbow Deviation")
ax2[1].set_ylabel("Deviation (°)")
ax2[1].set_title("Elbow")
ax2[1].set_xlabel("Time (s)")
ax2[1].legend()
ax2[1].grid(True)
plt.tight_layout()
plt.savefig(os.path.join(here, 'joint_deviation_plot.png'))
print("Plot saved to joint_deviation_plot.png")

# conds = {
#     'C0': dict(tau_s=2.0, tau_e=0.0, lock_s=False, lock_e=False),
#     'C1': dict(tau_s=2.0, tau_e=0.0, lock_s=False, lock_e=True),
#     'C2': dict(tau_s=2.0, tau_e=2.0, lock_s=False, lock_e=False),
#     'C3': dict(tau_s=0.0, tau_e=2.0, lock_s=True,  lock_e=False),
#     'C4': dict(tau_s=0.0, tau_e=0.0, lock_s=True,  lock_e=True),
#     'C5': dict(tau_s=0.0, tau_e=0.0, lock_s=True,  lock_e=True),
#     'C6': dict(tau_s=0.0, tau_e=2.0, lock_s=False, lock_e=False),
# }
