from mode_manager import RecordedTrajectory
from utils import compress_with_indices

file = 'trajectory_data/trajectory.json'

trajectory = RecordedTrajectory(file)

# a = trajectory.get_task_pose(0)

# is_cont, joint, task, tool, task_time = trajectory.get_target(0)
# a = [True, True, True, True, False, False, False, True, True, True, False, False, True, True, True]
# b = [False, True, True, True, False, False, False, True, True, True, False, False, True, True, True]

# aa = compress_with_indices(a)
# bb = compress_with_indices(b)

# print(aa)
# print(bb)


print("Done")