from docutils import SettingsSpec
from evo.core import metrics
from evo.tools import log
log.configure_logging(verbose=True, debug=True,silent=False)
import pprint
import numpy as np
from evo.tools import plot
import matplotlib.pyplot as plt
from evo.tools.settings import SETTINGS
SETTINGS.plot_usetex = False
from evo.tools import file_interface
from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr, serialize_cdr
from rosbags.serde.serdes import cdr_to_ros1

#with Rosbag2Reader('/opt/my_workspace/ros2-slam-ekf-application/ros2_ws/result/autobin_result/output_25_jul/d1_chca_7/chca') as reader:
#    ref_pose = file_interface.read_bag_trajectory(reader, "/ref_pose")
#    current_pose = file_interface.read_bag_trajectory(reader, "/current_pose")

ref_file = "/opt/my_workspace/ros2-slam-ekf-application/ros2_ws/result/autobin_result/output_25_jul/d1_chca_7/ref_pose.kitti"
current_pose_file ="/opt/my_workspace/ros2-slam-ekf-application/ros2_ws/result/autobin_result/output_25_jul/d1_chca_7/current_pose.kitti"
ref_pose = file_interface.read_kitti_poses_file(ref_file)
current_pose = file_interface.read_kitti_poses_file(current_pose_file)
from evo.core import sync
max_diff = 0.01
ref_pose, current_pose = sync.associate_trajectories(ref_pose, current_pose)

fig = plt.figure()
label = {
    "estimate pose": current_pose,
    "reference": ref_pose,
}

plot.trajectories(fig, label, plot.PlotMode.xy)
plt .show()

pose_relation = metrics.PoseRelation.full_transformation
data = (ref_pose, current_pose)

ape_metric = metrics.APE(pose_relation)
ape_metric.process_data(data)

ape_stat = ape_metric.get_statistic(metrics.StatisticsType.rmse)
print("rmse")
print(ape_stat)

ape_stats = ape_metric.get_all_statistics()
pprint.pprint(ape_stats)

seconds_from_start = [t - current_pose.timestamps[0] for t in current_pose.timestamps]
fig = plt.figure()
plot.error_array(fig.gca(), ape_metric.error, x_array=seconds_from_start,
                 statistics={s:v for s,v in ape_stats.items() if s != "sse"},
                 name="APE", title="APE w.r.t. " + ape_metric.pose_relation.value, xlabel="$t$ (s)")
plt.show()

plot_mode = plot.PlotMode.xy
fig = plt.figure()
ax = plot.prepare_axis(fig, plot_mode)
plot.traj(ax, plot_mode, ref_pose, '--', "gray", "reference")
plot.traj_colormap(ax, current_pose, ape_metric.error, 
                   plot_mode, min_map=ape_stats["min"], max_map=ape_stats["max"])
ax.legend()
plt.show()