import matplotlib.pyplot as plt
import numpy as np

def read_file():
    with open("run_1p_reorder.txt") as f:
        data = f.read()

        data = data.split("\n")
        data = [x.split(" ") for x in data]
    return data

def get_data_points(marker, data):
    min_data = 100
    res = []
    for ele in data:
        if (len(ele) < 2):
            continue
        ele[3] = float(ele[3])
        min_data = min(min_data, ele[3])
        if marker in ele[1]:
            ele[2] = int(ele[2])
            res += [ele]
    print("Data Recvd: >= ", min_data)
    return res

def aggregate_udp_points(udp_data_points):
    dic = {}
    for ele in udp_data_points:
        if ele[0] not in dic:
            dic[ele[0]] = ele
        else:
            dic[ele[0]] = [ele[0], ele[1], dic[ele[0]][2]+ele[2], dic[ele[0]][3]+ele[3]]
    return list(dic.values())

def plot_cdfs(tcp_data_points, udp_data_points):
    # Extract time values from the lists
    time_values_tcp = [item[2] for item in tcp_data_points]
    time_values_udp = [item[2] for item in udp_data_points]

    # Calculate the CDFs
    time_values_tcp_sorted = np.sort(time_values_tcp)
    time_values_udp_sorted = np.sort(time_values_udp)
    
    cdf_1 = np.arange(1, len(time_values_tcp_sorted) + 1) / len(time_values_tcp_sorted)
    cdf_2 = np.arange(1, len(time_values_udp_sorted) + 1) / len(time_values_udp_sorted)

    # Plot the CDFs
    plt.plot(time_values_udp_sorted, cdf_2, label='TCP+UDP')
    plt.plot(time_values_tcp_sorted, cdf_1, label='TCP+TCP')

    all = time_values_tcp_sorted + time_values_udp_sorted

    # Add labels and title
    # plt.xticks([min(all)-10, max(all)+10])
    plt.xlabel('Time (milliseconds)')
    plt.ylabel('CDF')
    # plt.title('Different Transport Schemes Under 0.5% Packet Loss\n(UDP Successfully Transfers >= 99.02% of data)')
    # plt.title('Different Transport Schemes Under 0.1% Packet Loss\n(UDP Successfully Transfers >= 99.65% of data)')
    plt.title('Different Transport Schemes Under 1% Packet Reordering')
    plt.legend()

    # Show the plot
    plt.savefig("cdf_1p_reorder.png")
    plt.clf()

def plot_medians(tcp_data_points, udp_data_points):
    time_values_tcp = [item[2] for item in tcp_data_points]
    time_values_udp = [item[2] for item in udp_data_points]

    medians = [np.median(time_values_tcp), np.median(time_values_udp)]
    labels = ["TCP Scheme", "TCP+UDP Scheme"]
    
    plt.bar(labels, medians)
    
    for i, value in enumerate(medians):
        plt.text(i, value, str(value), ha='center', va='bottom')

    plt.xlabel('Transport Scehmes')
    plt.ylabel('Median Latency (milliseconds)')
    plt.title('Latency Comparison')

    # Show the plot
    plt.savefig("median_0_5_loss.pdf")
    plt.clf()


def main():
    data = read_file()
    tcp_data_points = get_data_points("TCP+TCP", data)
    udp_data_points = aggregate_udp_points(get_data_points("TCP+UDP", data))
    # plot_medians(tcp_data_points.copy(), udp_data_points.copy())
    plot_cdfs(tcp_data_points, udp_data_points)
main()