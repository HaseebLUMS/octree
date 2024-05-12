import csv
import numpy as np
import matplotlib.pyplot as plt
import sys

prefixes = ["75", "76", "77", "78", "79"]

loss_rate = {
    "11": 0.5,
    "9": 5,
    "8": 0,
    "6": 1,
    "12": 0.5,
    "29": 1,
}

def read_csv(file_path):
    data = {}
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data[row[0]] = float(row[1])
    return data


def compare_schemes(files):
    for file1 in files:
        scheme1 = read_csv(file1)

        frames = sorted([int(x) for x in scheme1.keys()])

        n = len(frames)//2
        tcp_frames = frames[0:n]
        print(len(tcp_frames))
        print(tcp_frames)
        dg_frames = frames[n:]
        print(dg_frames)

        time1 = [scheme1.get(str(frame), 0) for frame in tcp_frames]
        time2 = [scheme1.get(str(frame), 0) for frame in dg_frames]

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
        plt.savefig(f"{file1.split('/')[1]}.pdf")

        # diff = np.sort(diff)
        # cdf_diff = np.arange(1, len(diff) + 1) / len(diff)

        # plt.figure()  # CDFs
        # plt.plot(diff, cdf_diff, color='blue')

        # plt.xlabel('Time Savings (ms)')
        # plt.ylabel('CDF')
        # plt.grid(True)
        # plt.savefig(f"{files[0].split('/')[1]}_{prefixes[0]}_cdf.pdf")

# if len(sys.argv) > 1:
#     prefix = str(sys.argv[1])

# Paths to the CSV files
# file1_path = f"data/{prefix}time.csv"
# compare_schemes(file1_path)

files = []
for prefix in prefixes:
    p = f"data/{prefix}e2etime.csv"
    files.append(p)
compare_schemes(files)
