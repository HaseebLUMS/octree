import csv
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import math

# function to read a CSV
def read_csv(file_path):
    data = {}
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data[row[0]] = float(row[1])
    return data

# function to compare schemes
def compare_schemes(csv_file, expected_frames, outfile):
    print("Working on file:", csv_file, "Expected reliable frames:", expected_frames)
    scheme1 = read_csv(csv_file)
    frames = sorted([int(x) for x in scheme1.keys()])
    n = len(frames)//2
    tcp_frames = frames[0:n]
    print("Reliable Frames found:", len(tcp_frames))
    print(tcp_frames)
    dg_frames = frames[n:]
    print("Unreliable Frames found:", len(dg_frames))        
    print(dg_frames)

    time1 = [scheme1.get(str(frame), 0) for frame in tcp_frames]
    time2 = [scheme1.get(str(frame), 0) for frame in dg_frames]

    diff = [x[0]-x[1] for x in zip(time1, time2)]
    print(diff)
    colors = ['red' if val < 0 else 'blue' for val in diff]

    plt.figure()  # Bar Plots
    plt.bar(range(len(diff)), diff, color=colors)

    t = "Counted Per Frame"
    if "e2e" in csv_file:
        t = "Counted End To End"
    plt.title(f"How Much Late A Reliable Frame Is?\n{t}")
    plt.xlabel("Frame Number")
    plt.ylabel("Time (ms)")
    print("Check file:", outfile)
    plt.savefig(outfile)

    # diff = np.sort(diff)
    # cdf_diff = np.arange(1, len(diff) + 1) / len(diff)

    # plt.figure()  # CDFs
    # plt.plot(diff, cdf_diff, color='blue')

    # plt.xlabel('Time Savings (ms)')
    # plt.ylabel('CDF')
    # plt.grid(True)
    # plt.savefig(f"{files[0].split('/')[1]}_{prefixes[0]}_cdf.pdf")

# read input 
if len(sys.argv) == 10:
    QUICHE_CC = sys.argv[1]
    delay = sys.argv[2]
    loss = sys.argv[3]
    rate = sys.argv[4]
    prefix   = sys.argv[5]    
    FRAME_SIZE = sys.argv[6]
    DATA_SIZE   = sys.argv[7]
    BUFFER_SIZE = sys.argv[8]
    RUN_ID      = sys.argv[9]    
else: 
    print("Usage:", sys.argv[0], "QUICHE_CC delay loss rate prefix FRAME_SIZE DATA_SIZE BUFFER_SIZE RUN_ID")
    sys.exit(-1)

# verify folder exists 
if BUFFER_SIZE != "-1":
    base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{BUFFER_SIZE}/{FRAME_SIZE}-{DATA_SIZE}-{delay}-{loss}-{rate}"
else:
    base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{FRAME_SIZE}-{DATA_SIZE}-{delay}-{loss}-{rate}"

if not os.path.exists(base_folder):
    print("Something is wrong. Folder", base_folder, "does not exists! STOPPING!")
    sys.exit(-1)

# collect files to analyze
csv_file = f"{base_folder}/{prefix}/{prefix}e2etime.csv"
outfile = f"{base_folder}/{prefix}/{prefix}plot.pdf"
if not os.path.exists(csv_file):
    print("CSV File missing:", csv_file)
    sys.exit(-1)

# plot the results 
expected_frames = math.ceil(float(DATA_SIZE)/float(FRAME_SIZE))
compare_schemes(csv_file, expected_frames, outfile)
