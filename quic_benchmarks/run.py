import csv
import numpy as np
import matplotlib.pyplot as plt

prefix = "2"


def read_csv(file_path):
    data = {}
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data[row[0]] = float(row[1])
    return data

def compare_schemes(file1, file2):
    scheme1 = read_csv(file1)
    scheme2 = read_csv(file2)

    frames = sorted(set(scheme1.keys()) | set(scheme2.keys()))
    time1 = [scheme1.get(frame, 0) for frame in frames]
    time2 = [scheme2.get(frame, 0) for frame in frames]

    ind = np.arange(len(frames))
    width = 0.35

    fig, ax = plt.subplots()
    rects1 = ax.bar(ind - width/2, time1, width, label=file1.split("/")[-1])
    rects2 = ax.bar(ind + width/2, time2, width, label=file2.split("/")[-1])

    ax.set_ylabel('Time (ms)')
    ax.set_xlabel('Frame Name')
    # ax.set_title('Comparison of Time Taken for Frame Types')
    ax.set_xticks(ind)
    ax.set_xticklabels(frames)
    ax.yaxis.set_major_locator(plt.MultipleLocator(300))
    ax.yaxis.grid(True, linestyle='-', alpha=0.5)


    legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=2)
    # legend.get_frame().set_alpha(0.1)

    plt.savefig(f"{prefix}.pdf")


# Paths to the CSV files
file1_path = f"data/{prefix}dg_time.csv"
file2_path = f"data/{prefix}stream_time.csv"

compare_schemes(file1_path, file2_path)
