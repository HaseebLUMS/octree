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
    "12": 0.5,
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
    print(diff)
    colors = ['red' if val < 0 else 'blue' for val in diff]

    plt.figure()  # Bar Plots
    plt.bar(range(len(diff)), diff, color=colors)

    t = "Counted Per Frame"
    if "e2e" in file1:
        t = "Counted End To End"
    plt.title(f"How Much Late A Reliable Frame Is?\n{t}")
    plt.xlabel("Frame Number")
    plt.ylabel("Time (ms)")
    plt.savefig(f"{file1.split('/')[1]}_{prefix}.pdf")
    
    plt.figure()  # CDFs
    sorted_time1 = np.sort(time1)
    cdf_time1 = np.arange(1, len(sorted_time1) + 1) / len(sorted_time1)
    plt.plot(sorted_time1, cdf_time1, color='red', label='Time1 (TCP)')

    sorted_time2 = np.sort(time2)
    cdf_time2 = np.arange(1, len(sorted_time2) + 1) / len(sorted_time2)
    plt.plot(sorted_time2, cdf_time2, color='blue', label='Time2 (DG)')
    
    plt.xlabel('Time (ms)')
    plt.ylabel('CDF')
    plt.legend()
    plt.grid(True)

    plt.savefig(f"{file1.split('/')[1]}_{prefix}_cdf.pdf")

if len(sys.argv) > 1:
    prefix = str(sys.argv[1])

# Paths to the CSV files
file1_path = f"data/{prefix}time.csv"
compare_schemes(file1_path)

file1_path = f"data/{prefix}e2etime.csv"
compare_schemes(file1_path)
