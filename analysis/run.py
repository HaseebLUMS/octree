import pandas as pd
import matplotlib.pyplot as plt

cpus_filename = "data/baseline_cpu.csv"
storage_filename = "data/baseline_storage.csv"

font_size = 15

markers = ['o', '^', 'D', 's', 'v', '<', '>', 'p', '*', 'h']

def plot_difference(filename, label, ind):
    df = pd.read_csv(filename, header=None, names=['version', 'category', 'usage'])

    df = df[df['category'] == "soldier"]

    # Calculate baseline (version 1) usage for each category
    baseline = df[df['version'] == 1].set_index('category')['usage']

    if (ind == 2):
        baseline = baseline.add(30)

    # Calculate the usage difference compared to baseline for each row
    df['usage_diff'] = df.apply(lambda row: row['usage'] / baseline[row['category']], axis=1)

    print(df['usage_diff'])

    # Plot each category separately
    for category in df['category'].unique():
        subset = df[df['category'] == category]
        plt.plot(subset['version'], subset['usage_diff'], marker=markers[ind], label=label)


plt.figure(figsize=(10, 6))

plot_difference(cpus_filename, "CPU Savings", 0)
plot_difference(storage_filename, "Storage Savings", 1)
plot_difference(cpus_filename, "CPU Savings Accounting\nfor QUIC overhead", 2)

current_yticks = plt.gca().get_yticks()

# Create new y-tick labels with 'x' suffix
ytick_labels = [f'{float(y)}x' for y in current_yticks]
plt.gca().set_yticklabels(ytick_labels)

plt.rcParams.update({'font.size': font_size})  # Update font size for all elements

plt.xticks(fontsize=font_size)
plt.yticks(fontsize=font_size)

plt.xlabel('Supported Point Cloud Versions', fontsize=font_size)
plt.ylabel('Resource Savings Over The Baseline', fontsize=font_size)
plt.title('CPU and Storage Savings Of Octavius', fontsize=font_size)
plt.legend(fontsize=font_size)
plt.savefig(f"figs/resource_savings.pdf")