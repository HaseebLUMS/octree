import csv
import numpy as np
import matplotlib.pyplot as plt
import sys

prefix = "6"

loss_rate = {
    "11": 0.5,
    "9": 5,
    "8": 0,
    "6": 1,
}

def read_csv(file_path):
    data = {}
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data[row[0]] = float(row[1])
    return data

def compare_schemes(file1):
    scheme1 = read_csv(file1)

    frames = sorted(set(scheme1.keys()))

    n = len(frames)//2
    tcp_frames = frames[0:n]
    print(len(tcp_frames))
    dg_frames = frames[n:]
    print(len(dg_frames))

    time1 = [scheme1.get(frame, 0) for frame in tcp_frames]
    time2 = [scheme1.get(frame, 0) for frame in dg_frames]

    diff = [x[0]-x[1] for x in zip(time1, time2)]
    print("Median frame delay:", np.median(diff))

    ind = np.arange(len(tcp_frames))
    width = 0.35

    fig, ax = plt.subplots()
    rects1 = ax.bar(ind - width/2, time1, width, label="Reliable Frames")
    rects2 = ax.bar(ind + width/2, time2, width, label="Best Effort Frames")

    ax.set_ylabel('Time (ms)')
    ax.set_xlabel('Frame Name')
    ax.set_xticks(ind)
    ax.yaxis.set_major_locator(plt.MultipleLocator(500))
    ax.yaxis.grid(True, linestyle='-', alpha=0.5)

    ax.set_title(f"{loss_rate[prefix]}% Packet Loss Rate | Observed Median Frame Delay: {round(np.median(diff), 1)}ms")

    legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=2)
    plt.savefig(f"{prefix}.pdf")

if len(sys.argv) > 1:
    prefix = str(sys.argv[1])

# Paths to the CSV files
file1_path = f"data/{prefix}time.csv"

compare_schemes(file1_path)
