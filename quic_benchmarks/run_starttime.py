import matplotlib.pyplot as plt
import numpy as np

# Data from your file
start_times = {}
end_times = {}

prefix = 40

file1 = f"data/{prefix}starttime.csv"
file2 = f"data/{prefix}time.csv"


with open(file1, "r") as file:
    for line in file:
        frame_number, start_time = map(float, line.strip().split(","))
        start_times[int(frame_number)] = int(start_time)

with open(file2, "r") as file:
    for line in file:
        frame_number, time = map(float, line.strip().split(","))
        end_times[int(frame_number)] = int(start_times[frame_number] + time)

frames = sorted([int(x) for x in start_times.keys()])

n = len(frames)//2
tcp_frames = frames[0:n]
dg_frames = frames[n:]

# Plotting the timeline
plt.figure(figsize=(10, 6))

marker_size = 20

tcp_colors = []

for frame in tcp_frames:
    point = plt.scatter(start_times[frame], frame, marker='o', s=marker_size)
    plt.text(start_times[frame], frame, str(frame), ha='right', va='bottom')  # Adding frame number label
    color = point.get_facecolor().flatten()
    tcp_colors.append(color)
    
    plt.scatter(end_times[frame], frame, marker='x', s=marker_size, color = color)
    # plt.text(end_times[frame], frame, str(frame), ha='right', va='bottom')  # Adding frame number label

for ind, frame in enumerate(dg_frames):
    point = plt.scatter(start_times[frame], frame, marker='o', s=marker_size, color=tcp_colors[ind])
    plt.text(start_times[frame], frame, str(frame - len(tcp_frames)), ha='right', va='bottom')  # Adding frame number label
    color = point.get_facecolor().flatten()
    
    plt.scatter(end_times[frame], frame, marker='x', s=marker_size, color=color)
    # plt.text(end_times[frame], frame, str(frame - len(tcp_frames)), ha='right', va='bottom')  # Adding frame number label

plt.xlabel('Start Time (milliseconds)')
plt.ylabel('Frame Number')
plt.title('Video Frame Timeline')
plt.grid(True)
plt.tight_layout()
plt.savefig(f"{file1.split('/')[1]}_{prefix}.pdf")
