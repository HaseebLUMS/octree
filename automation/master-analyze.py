import csv
import numpy as np
import sys
import os
import math
import matplotlib
from matplotlib import rcParams
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype']  = 42
matplotlib.use('Agg')
rcParams.update({'figure.autolayout': True})
rcParams.update({'errorbar.capsize': 2})
import matplotlib.pyplot as plt
from pylab import *
from matplotlib import ticker
import matplotlib.patches as patches

# increase font for plotting
font = {'weight' : 'medium',
        'size'   : 14}
matplotlib.rc('font', **font)

# function to read a CSV
def read_csv(file_path):
    data = {}
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data[row[0]] = float(row[1])
    return data

# function to compare schemes
def analyze(csv_file, expected_frames):
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
    #print(diff)    
    return diff, len(tcp_frames), len(dg_frames)

# simple helper to lighten a color
def lighten_color(color, amount=0.5):
    import matplotlib.colors as mc
    import colorsys
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])


# function for setting the colors of the box plots pairs
def setBoxColors(bp, c, h):
    for i in range(len(bp['boxes'])):
        bp['boxes'][i].set(facecolor = c)
        bp['boxes'][i].set(hatch = h)
        setp(bp['medians'][i],   color = 'black')
    for i in range(0, len(bp['caps']), 2):    
        setp(bp['caps'][i],      color = c)
        setp(bp['caps'][i + 1],      color = c)                                
        setp(bp['whiskers'][i],  color = c, linestyle =  'dashed')
        setp(bp['whiskers'][i + 1],  color = c, linestyle =  'dashed')                                
        
# global parameters
color_list  = ['red', 'blue', 'green', 'magenta', 'black', 'purple', 'orange', 'yellow', 'cyan', 'gray',lighten_color('red'), lighten_color('blue'), lighten_color('green'), lighten_color('magenta'), lighten_color('black'), lighten_color('purple'), lighten_color('orange') ]    # colors supported
style_list  = ['solid', 'dashed', 'dotted']              # styles of plots  supported
marker_list = ['v', 'h', 'D', '8', '+' ]                 # list of markers supported
space_between_boxplots = 5
boxplot_width = 3
ext = 'pdf'
color_dict = {}
color_dict['QUICHE_CC_RENO']  = lighten_color('red')
color_dict['QUICHE_CC_CUBIC'] = lighten_color('blue')
color_dict['QUICHE_CC_BBR']   = lighten_color('green')
color_dict['QUICHE_CC_BBR2']  = lighten_color('magenta')
color_dict['QUICHE_CC_CUBIC_HYSTART'] = 'blue'
BUFFER_SIZE=-1

# read and check input 
if len(sys.argv) == 9 or len(sys.argv) == 10:
    QUICHE_CC_list = sys.argv[1].split(',')
    latency_list = sys.argv[2].split(',')
    loss_list  = sys.argv[3].split(',')
    rate       = sys.argv[4]
    NUM_REPS   = int(sys.argv[5])
    FRAME_SIZE = sys.argv[6]
    DATA_SIZE  = sys.argv[7]
    RUN_ID     = sys.argv[8]
    if len(sys.argv) == 10:
        BUFFER_SIZE = 1000*int(sys.argv[9]) #KB
else: 
    print("Usage:", sys.argv[0], "QUICHE_CC_list delay_list loss_list rate NUM_REPS FRAME_SIZE DATA_SIZE RUN_ID [BUFFER_SIZE]")
    sys.exit(-1)

# derive expected frames and TOT_SIZE
expected_frames = math.ceil(float(DATA_SIZE)/float(FRAME_SIZE))
TOT_SIZE = 2 * int(DATA_SIZE)

# manually generate legend
c = 0
custom_lines = []
custom_labels = []
for QUICHE_CC in QUICHE_CC_list:   
    custom_lines.append(patches.Rectangle((0,0),2,2, edgecolor = 'black', facecolor = color_dict[QUICHE_CC]))
    custom_labels.append(QUICHE_CC)
    c += 1 
        
# create figure handlers
fig_final_lat  = plt.figure()
fig_median_lat = plt.figure()
fig_loss       = plt.figure()
fig_rate       = plt.figure()
fig_num_unreliable = plt.figure()
fig_num_reliable = plt.figure()
fig_cpu = plt.figure()

# Iterate over latencies
curr_pos = 0
xtick_boxplot_loc = []
xtick_boxplot_labels = []
for latency in latency_list:
    for loss in loss_list:
        c = 0
        for QUICHE_CC in QUICHE_CC_list:
            final_lat       = []
            median_lat      = []
            unreliable_loss = []
            unreliable_duration = []
            reliable_loss     = []
            reliable_duration = []
            num_reliable_frames = []
            num_unreliable_frames = []        
            median_cpu = []

            # verify folder exists 
            if BUFFER_SIZE == -1: 
                base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{FRAME_SIZE}-{DATA_SIZE}-{latency}-{loss}-{rate}"
            else:
                base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{BUFFER_SIZE}/{FRAME_SIZE}-{DATA_SIZE}-{latency}-{loss}-{rate}"                    
            if not os.path.exists(base_folder):
                print("WARNING. Folder", base_folder, "does not exists!")
                continue
            
            # get TC info from the primer             
            tc_stats = f"{base_folder}/1/1tc-stats"
            if not os.path.exists(tc_stats):
                print("tc_stats file missing:", tc_stats)
                continue               
            print("[TC] Working on file:", tc_stats)                     
            with open(tc_stats, 'r') as file:
                line = file.readline().strip()
            array = line.split('-')
            prev_bytes_sent       = float(array[0])
            prev_bytes_dropped    = float(array[1])
            prev_bytes_overlimits = float(array[2])
            prev_bytes_requeues   = float(array[3])
            print("[TC] Run: PRIMER Sent:", prev_bytes_sent, "Dropped:", prev_bytes_dropped, "Overlimits:", prev_bytes_overlimits, "Requeues:",  prev_bytes_requeues)

            # iterate on non-primer runs
            for i in range(2, NUM_REPS + 1): #NOTE: skipping the "primer"
                exp_id = i              
                
                # verify CSV file exists 
                csv_file = f"{base_folder}/{i}/{i}e2etime.csv"
                if not os.path.exists(csv_file):
                    print("CSV File missing:", csv_file)
                    continue

                # verify client log exists 
                client_log = f"{base_folder}/{i}/client-log-{exp_id}"                
                if not os.path.exists(client_log):
                    print("client_log file missing:", client_log)
                    continue

                # verify no issue on CWDN
                INITIAL_WINDOW = 13500
                num_low_window = 0 
                num_exc = 0
                
                # check CWN on qlog analysis
                cwd_rtt = f"{base_folder}/{i}/{exp_id}cwd_rtt"                        
                if os.path.exists(cwd_rtt):
                    with open(cwd_rtt, 'r') as file:
                        for line in file:
                            fields = line.split(" ")
                            try:
                                cwnd = float(fields[1])
                                if cwnd < INITIAL_WINDOW:
                                    num_low_window += 1 
                            except ValueError as e:
                                num_exc += 1
                else: # check on server log 
                    server_log = f"{base_folder}/{i}/server-log-{exp_id}"
                    print("Working on server log:", server_log)
                    if os.path.exists(server_log):
                        with open(server_log, 'r') as file:
                            for line in file:
                                if 'INFO' not in line:
                                    continue
                                fields = line.split(",")
                                try:
                                    cwnd = float(fields[3])
                                    if cwnd < INITIAL_WINDOW:
                                        num_low_window += 1 
                                except ValueError as e:
                                    num_exc += 1
                if num_low_window > 200:                
                    print("skipping results since something wrong on initial window. Found num_low_window:", num_low_window, "file:", cwd_rtt)
                    # TODO: do a plot of CWND and see what happens...
                    continue

                # derive loss and duration for reliable/unreliable data                 
                with open(client_log, 'r') as file:
                    for line in file:
                        #if "Total Received (% out of expected)" in line:
                        if "Unreliably Received" in line:
                            #val = float(line.split(':')[1].strip())
                            val = float(line.split(':')[1].split("(")[0].strip())                            
                            print("Unreliable Percentage:", val)
                            unreliable_loss.append((1 - val)*100)
                            val = float(line.split(':')[1].split(" ")[-2])
                            print("Unreliable Duration:", val)
                            unreliable_duration.append((TOT_SIZE*8)/(val/1000))
                            
                        if "Reliably Received" in line:
                            val = float(line.split(':')[1].lstrip().split(" ")[0].strip())
                            print("Reliable Percentage:", val)
                            reliable_loss.append((1 - val)*100)
                            val = float(line.split(':')[1].split(" ")[-2])
                            print("Reliable Duration:", val)
                            reliable_duration.append(val/1000)
                
                # get TC stats 
                tc_stats = f"{base_folder}/{i}/{exp_id}tc-stats"
                if not os.path.exists(tc_stats):
                    print("tc_stats file missing:", tc_stats)
                    continue                    
                print("[TC] Working on file:", tc_stats)
                with open(tc_stats, 'r') as file:
                    line = file.readline().strip()
                array = line.split('-')
                bytes_sent       = float(array[0]) - prev_bytes_sent
                bytes_dropped    = float(array[1]) - prev_bytes_dropped
                bytes_overlimits = float(array[2]) - prev_bytes_overlimits
                bytes_requeues   = float(array[3]) - prev_bytes_requeues
                prev_bytes_sent       = float(array[0])
                prev_bytes_dropped    = float(array[1])
                prev_bytes_overlimits = float(array[1])
                prev_bytes_requeues   = float(array[3])
                print("[TC] Run:", i, "Sent:", bytes_sent, "Dropped:", bytes_dropped, "Overlimits:", bytes_overlimits, "Requeues:",  bytes_requeues)

                # derive final latency
                lat, num_reliable, num_unreliable = analyze(csv_file, expected_frames)
                final_lat.append((np.max(lat)/unreliable_duration[-1]/1000)*100)
                #final_lat.append(lat[-1])
                median_lat.append((np.median(lat)/unreliable_duration[-1]/1000)*100)
                #median_lat.append(np.median(lat))
                num_reliable_frames.append(num_reliable)
                num_unreliable_frames.append(num_unreliable)

                # get CPU info 
                cpu_log = f"{base_folder}/{i}/{exp_id}cpu-log"
                if not os.path.exists(cpu_log):
                    print("cpu_log file missing:", cpu_log)
                    continue                    
                print("[TC] Working on file:", cpu_log)
                cpu_vals = []
                with open(cpu_log, 'r') as file:
                    for line in file:
                        if 'Monitoring' in line:
                            continue
                        cpu_vals.append(float(line.strip()))
                    median_cpu.append(np.median(cpu_vals))

            # plot median CPU
            print(median_cpu)
            plt.figure(fig_cpu.number)        
            bp = plt.boxplot(median_cpu, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot num reliable frames 
            print(num_reliable_frames)
            plt.figure(fig_num_reliable.number)        
            bp = plt.boxplot(num_reliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot num unreliable frames 
            print(num_unreliable_frames)
            plt.figure(fig_num_unreliable.number)        
            bp = plt.boxplot(num_unreliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot final lat as boxplot
            print(final_lat)
            plt.figure(fig_final_lat.number)        
            bp = plt.boxplot(final_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot median lat as boxplot
            print(median_lat)
            plt.figure(fig_median_lat.number)        
            bp = plt.boxplot(median_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")
            
            # plot unreliable duration
            print(unreliable_duration)
            plt.figure(fig_rate.number)        
            bp = plt.boxplot(unreliable_duration, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot unreliable losses
            print(unreliable_loss)
            plt.figure(fig_loss.number)        
            bp = plt.boxplot(unreliable_loss, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")            

            # update counters and positions
            curr_pos += boxplot_width            
            c += 1 
        xtick_boxplot_loc.append(curr_pos - boxplot_width*len(QUICHE_CC_list)/2)
        xtick_boxplot_labels.append(str(latency)+"ms\n" + str(loss) + "%")
        curr_pos += space_between_boxplots
    curr_pos += space_between_boxplots
       
# make folder if needed
plot_path = 'plots/' + RUN_ID
os.makedirs(plot_path, exist_ok = True)

# save boxplots for (final) latency
plt.figure(fig_final_lat.number)            
tentative_font = 10     # font 
rot = 10                # rotation of labels
num_columns = 1         # number of columns in legend
location = 'upper right'
plot_file = plot_path + '/comp-boxplots-final-latency.' + ext
plt.ylabel('Latency Savings (%)')
ax = plt.gca()   
ax.set_yscale('log')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
exp_info = f"FrameSize:{FRAME_SIZE}MB TotData:{TOT_SIZE}MB (50/50) Rate:{rate}Mpbs ID:{RUN_ID}"
if BUFFER_SIZE > 0: 
    bf_size = BUFFER_SIZE/1000
    exp_info = f"Frame:{FRAME_SIZE}MB TotData:{TOT_SIZE}MB (50/50) Rate:{rate}Mpbs ID:{RUN_ID} Buffer:{bf_size}"
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for (median) CPU
plt.figure(fig_cpu.number)            
plot_file = plot_path + '/comp-boxplots-median-CPU.' + ext
plt.ylabel('CPU (%)')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for (median) latency
plt.figure(fig_median_lat.number)            
plot_file = plot_path + '/comp-boxplots-median-latency.' + ext
plt.ylabel('Latency Savings (%)')
ax = plt.gca()   
ax.set_yscale('log')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for num reliable frames
plt.figure(fig_num_unreliable.number)            
plot_file = plot_path + '/comp-boxplots-num-unreliable.' + ext
plt.ylabel('Num Reliable Frames (#)')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for num unreliable frames
plt.figure(fig_num_reliable.number)            
plot_file = plot_path + '/comp-boxplots-num-reliable.' + ext
plt.ylabel('Num Reliable Frames (#)')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for duration
plt.figure(fig_rate.number)            
plot_file = plot_path + '/comp-boxplots-rate.' + ext
plt.ylabel('Rate (Mbps)')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for losses
plt.figure(fig_loss.number)            
plot_file = plot_path + '/comp-boxplots-loss.' + ext
plt.ylabel('Unreliable Loss (%)')
#ax = plt.gca()   
#ax.set_yscale('log')
plt.grid(True)
plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  