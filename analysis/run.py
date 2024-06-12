import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

cpus_filename = "data/baseline_cpu.csv"
storage_filename = "data/baseline_storage.csv"

font_size = 14
categories = ['soldier', 'reddress', 'ricardo']
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

def plot_difference(filename, baseline_adjustment=0):
    df = pd.read_csv(filename, header=None, names=['version', 'category', 'usage'])
    df['usage'] = df['usage'].astype(float)

    # Calculate baseline (version 1) usage for each category
    baseline = df[df['version'] == 1].set_index('category')['usage'] + baseline_adjustment

    # Calculate the usage difference compared to baseline for each row and convert to percentage
    df['usage_diff'] = df.apply(lambda row: ((row['usage'] / baseline[row['category']]) - 1) * 100, axis=1)

    return df

# Process data
cpu_df = plot_difference(cpus_filename)
storage_df = plot_difference(storage_filename)
cpu_quic_df = plot_difference(cpus_filename, 30)

# Filter data to show only the last version
last_version = max(cpu_df['version'].max(), storage_df['version'].max(), cpu_quic_df['version'].max())
cpu_df_last_version = cpu_df[cpu_df['version'] == last_version]
storage_df_last_version = storage_df[storage_df['version'] == last_version]
cpu_quic_df_last_version = cpu_quic_df[cpu_quic_df['version'] == last_version]

# Merge dataframes and prepare for plotting
raw_dfs = [cpu_df, cpu_quic_df, storage_df]
dfs = [cpu_df_last_version, cpu_quic_df_last_version, storage_df_last_version]
labels = ["CPU Savings", "CPU Savings Accounting for QUIC overhead", "Storage Savings"]

# Preparing plot
fig, ax = plt.subplots(figsize=(12, 8))

# x locations for the bars
x = np.arange(len(categories))
bar_width = 0.25

# Plotting the bar for each version separately
for i, (df, label) in enumerate(zip(dfs, labels)):
    # Pivot table to align data for plotting
    pivot_df = df.pivot_table(index='version', columns='category', values='usage_diff').reindex(columns=categories).fillna(0)
    
    # Plotting the bar for the last version
    for version in pivot_df.index:
        usage_values = pivot_df.loc[version].values
        ax.bar(x + i * bar_width, usage_values, bar_width, color=colors[i], label=label)

# Adding red dotted lines within the bars for other versions
for i, raw_df in enumerate(raw_dfs):
    for j, category in enumerate(categories):
        for v in range(1, last_version+1):
            usage_diff = raw_df.loc[(raw_df['category'] == category) & (raw_df['version'] == v), 'usage_diff'].values
            if len(usage_diff) > 0:
                line_height = usage_diff[0]
                # Calculate the midpoint of the line
                midpoint_x = x[j] + (i - 0.5) * bar_width + bar_width / 2
                # Adjusting the x-position of the lines to align them at the center of each bar
                plt.hlines(y=line_height, xmin=x[j] + (i - 0.5) * bar_width, 
                           xmax=x[j] + (i + 0.5) * bar_width, color='red', linewidth=2, linestyle='dotted')
                # Adding text annotation for version at the midpoint of the line, but only for the first bar and category
                if j == 0 and i == 0:
                    extra = ""
                    if v > 1:
                        extra = "+"
                    plt.text(midpoint_x, line_height, f'{extra}v{v}', color='black', ha='center', va='bottom', fontsize=10)

# Setting labels and ticks
# ax.set_xlabel('Categories', fontsize=font_size)
ax.set_ylabel('Resource Savings Over The Baseline', fontsize=font_size)
ax.set_xticks(x + bar_width)
ax.set_xticklabels(categories, fontsize=font_size)
ax.legend(fontsize=font_size)

# Update y-tick labels to percentage
current_yticks = ax.get_yticks()
ytick_labels = [f'{int(y)}%' for y in current_yticks]
ax.set_yticklabels(ytick_labels)

plt.tight_layout()
plt.savefig("figs/resource_savings.pdf")
plt.show()
