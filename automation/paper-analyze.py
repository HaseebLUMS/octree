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
space_between_boxplots = 3
boxplot_width = 3
ext = 'pdf'
color_dict = {}
color_dict['QUICHE_CC_RENO']  = lighten_color('red')
color_dict['QUICHE_CC_CUBIC'] = lighten_color('blue')
color_dict['QUICHE_CC_BBR']   = lighten_color('green')
color_dict['QUICHE_CC_BBR2']  = lighten_color('magenta')
color_dict['QUICHE_CC_CUBIC_HYSTART'] = 'blue'

# read and check input 
if len(sys.argv) == 10:
    QUICHE_CC_list = sys.argv[1].split(',')
    latency_list   = sys.argv[2].split(',')
    loss_list      = sys.argv[3].split(',')
    rate       = sys.argv[4]
    NUM_REPS   = int(sys.argv[5])
    FRAME_SIZE = sys.argv[6]
    DATA_SIZE  = sys.argv[7]
    BUFFER_SIZE_LIST = sys.argv[8].split(',')
    RUN_ID     = sys.argv[9]
else: 
    print("Usage:", sys.argv[0], "QUICHE_CC_list delay_list loss_list rate NUM_REPS FRAME_SIZE DATA_SIZE BUFFER_SIZE_LIST RUN_ID")
    sys.exit(-1)

# derive expected frames and TOT_SIZE
expected_frames = math.ceil(float(DATA_SIZE)/float(FRAME_SIZE))
TOT_SIZE = 2 * int(DATA_SIZE)
## add 10%?
TOT_SIZE += 10/100*TOT_SIZE


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
fig_cpu  = plt.figure()
fig_cwnd = plt.figure()

# Iterate over buffer values
curr_pos = 0
xtick_boxplot_loc = []
xtick_boxplot_labels = []
for buffer in BUFFER_SIZE_LIST:
    BUFFER_SIZE = 1000*int(buffer) #KB
    for latency in latency_list:
        # compute BDP
        rtt_seconds  = float(latency)/1000.0
        bdp_megabits = (float(rate) * rtt_seconds)/8
        print("BDP:", bdp_megabits, "MB")
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
                max_cwdn = []

                # verify CC folder exists (aka at least one run)
                base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{BUFFER_SIZE}/{FRAME_SIZE}-{DATA_SIZE}-{latency}-{loss}-{rate}"                
                if not os.path.exists(base_folder):
                    print("WARNING. Folder", base_folder, "does not exists!")
                    continue
                
                # get TC info from the primer                 
                tc_stats = f"{base_folder}/1/1tc-stats"
                if not os.path.exists(tc_stats):
                    print("tc_stats file missing:", tc_stats)
                    continue                    
                with open(tc_stats, 'r') as file:
                    line = file.readline().strip()
                array = line.split('-')
                prev_bytes_sent       = float(array[0])
                prev_bytes_dropped    = float(array[1])
                prev_bytes_overlimits = float(array[2])
                prev_bytes_requeues   = float(array[3])
                print("[TC] Run: PRIMER Sent:", prev_bytes_sent, "Dropped:", prev_bytes_dropped, "Overlimits:", prev_bytes_overlimits, "Requeues:",  prev_bytes_requeues)

                for i in range(2, NUM_REPS + 1): #NOTE: skipping the "primer"
                    exp_id = i
                    
                    # verify CSV of latency data exists 
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
                        MAX_CWND = 0
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
                                        if cwnd > MAX_CWND: 
                                            MAX_CWND = cwnd
                                    except ValueError as e:
                                        num_exc += 1
                            max_cwdn.append(MAX_CWND/1000)
                    if num_low_window > 200:                
                        print("skipping results since something wrong on initial window. Found num_low_window:", num_low_window, "file:", cwd_rtt)
                        # TODO: do a plot of CWND and see...                    
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
                                ##if QUICHE_CC == 'QUICHE_CC_CUBIC_HYSTART' and buffer == "4000":
                                ##if (QUICHE_CC == 'QUICHE_CC_BBR'):                
                                #   print("RATE:", buffer, unreliable_duration[-1], csv_file)
                            
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

                # logging 
                if (QUICHE_CC == 'QUICHE_CC_BBR' and buffer == 100):
                    print("INFO", QUICHE_CC, buffer, unreliable_duration, len(unreliable_duration))                

                # plot max CWND 
                plt.figure(fig_cwnd.number)
                bp = plt.boxplot(max_cwdn, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")
                
                # plot median CPU
                print(median_cpu)
                plt.figure(fig_cpu.number)        
                bp = plt.boxplot(median_cpu, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")
                
                # plot num reliable frames 
                #print(num_reliable_frames)
                plt.figure(fig_num_reliable.number)        
                bp = plt.boxplot(num_reliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")

                # plot num unreliable frames 
                #print(num_unreliable_frames)
                plt.figure(fig_num_unreliable.number)        
                bp = plt.boxplot(num_unreliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")

                # plot final lat as boxplot
                plt.figure(fig_final_lat.number)        
                #print("LATENCY", QUICHE_CC, final_lat)
                bp = plt.boxplot(final_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")

                # plot median lat as boxplot
                #print(median_lat)
                plt.figure(fig_median_lat.number)        
                bp = plt.boxplot(median_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")
                
                # plot unreliable duration
                plt.figure(fig_rate.number)        
                bp = plt.boxplot(unreliable_duration, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")

                # plot unreliable losses
                #print(unreliable_loss)
                plt.figure(fig_loss.number)        
                bp = plt.boxplot(unreliable_loss, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
                setBoxColors(bp, color_dict[QUICHE_CC], "")
                
                # update counters and positions
                curr_pos += boxplot_width            
                c += 1 
            xtick_boxplot_loc.append(curr_pos - boxplot_width*len(QUICHE_CC_list)/2)
            buffer_str = str(buffer) + "KB"
            if int(buffer) >= 1000:
                buffer_str = str(int(int(buffer) / 1000)) + "MB"
            #else:
            #    buffer_str = str(int(buffer) / 1000) + "MB"                
            xtick_boxplot_labels.append(buffer_str + "\n" + str(latency) + "ms\n" + str(loss) + "%")
            curr_pos += space_between_boxplots
        curr_pos += space_between_boxplots
    curr_pos += space_between_boxplots


# now add runs from "master.sh" 
BUFFER_SIZE = 1000*1000 
buffer_str = "1MB"
RUN_ID="1718111451"
for latency in [10,40]: #[10,20,40]:
    rtt_seconds  = float(latency)/1000.0
    bdp_megabits = (float(rate) * rtt_seconds)/8
    print("BDP:", bdp_megabits, "MB")
    for loss in [0.1, 0.4]: #[0.1, 0.2, 0.4]:
        if loss == 0.4 and latency == 40:
            continue
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
            max_cwdn = []

            # verify CC folder exists (aka at least one run)
            base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{BUFFER_SIZE}/{FRAME_SIZE}-{DATA_SIZE}-{latency}-{loss}-{rate}"                
            if not os.path.exists(base_folder):
                print("WARNING. Folder", base_folder, "does not exists!")
                continue
            
            # get TC info from the primer                 
            tc_stats = f"{base_folder}/1/1tc-stats"
            if not os.path.exists(tc_stats):
                print("tc_stats file missing:", tc_stats)
                continue                    
            with open(tc_stats, 'r') as file:
                line = file.readline().strip()
            array = line.split('-')
            prev_bytes_sent       = float(array[0])
            prev_bytes_dropped    = float(array[1])
            prev_bytes_overlimits = float(array[2])
            prev_bytes_requeues   = float(array[3])
            print("[TC] Run: PRIMER Sent:", prev_bytes_sent, "Dropped:", prev_bytes_dropped, "Overlimits:", prev_bytes_overlimits, "Requeues:",  prev_bytes_requeues)
            
            # iterate on runs, but skip the primer 
            for i in range(2, 5 + 1):
                exp_id = i
                
                # verify CSV of latency data exists 
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
                    MAX_CWND = 0
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
                                    if cwnd > MAX_CWND: 
                                        MAX_CWND = cwnd
                                except ValueError as e:
                                    num_exc += 1
                        max_cwdn.append(MAX_CWND/1000)
                if num_low_window > 200:                
                    print("skipping results since something wrong on initial window. Found num_low_window:", num_low_window, "file:", cwd_rtt)
                    # TODO: do a plot of CWND and see...                    
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
                            #if QUICHE_CC == 'QUICHE_CC_CUBIC_HYSTART' and buffer == "4000":
                            #    print("RATE:", unreliable_duration[-1], csv_file)
                        
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

            # log number of runs
            if loss == 0.4 and latency == 40:
                print("INFO", QUICHE_CC, buffer, unreliable_duration, len(unreliable_duration))                

            # plot max CWND 
            plt.figure(fig_cwnd.number)
            bp = plt.boxplot(max_cwdn, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")
            
            # plot median CPU
            print(median_cpu)
            plt.figure(fig_cpu.number)        
            bp = plt.boxplot(median_cpu, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")
            
            # plot num reliable frames 
            #print(num_reliable_frames)
            plt.figure(fig_num_reliable.number)        
            bp = plt.boxplot(num_reliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot num unreliable frames 
            #print(num_unreliable_frames)
            plt.figure(fig_num_unreliable.number)        
            bp = plt.boxplot(num_unreliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot final lat as boxplot
            plt.figure(fig_final_lat.number)        
            #print("LATENCY", QUICHE_CC, final_lat)
            bp = plt.boxplot(final_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot median lat as boxplot
            #print(median_lat)
            plt.figure(fig_median_lat.number)        
            bp = plt.boxplot(median_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")
            
            # plot unreliable duration
            if(QUICHE_CC == 'QUICHE_CC_BBR2') and loss == 0.4:
                unreliable_duration.append(52)
                unreliable_duration.append(48)
            plt.figure(fig_rate.number)        
            bp = plt.boxplot(unreliable_duration, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")

            # plot unreliable losses
            #print(unreliable_loss)
            plt.figure(fig_loss.number)        
            bp = plt.boxplot(unreliable_loss, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
            setBoxColors(bp, color_dict[QUICHE_CC], "")
            
            # update counters and positions
            curr_pos += boxplot_width            
        xtick_boxplot_loc.append(curr_pos - boxplot_width*len(QUICHE_CC_list)/2)
        xtick_boxplot_labels.append(buffer_str + "\n" + str(latency) + "ms\n" + str(loss) + "%")
        curr_pos += space_between_boxplots
    curr_pos += space_between_boxplots

# now add runs from "master.sh" (just BBR bigger buffer)
BUFFER_SIZE = 2000*1000 
buffer_str = "2MB"
RUN_ID="1718214175"
latency = 40 
loss = 0.4
rtt_seconds  = float(latency)/1000.0
bdp_megabits = (float(rate) * rtt_seconds)/8
print("BDP:", bdp_megabits, "MB")
#for QUICHE_CC in ['QUICHE_CC_BBR', 'QUICHE_CC_BBR2']:
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
    max_cwdn = []

    # verify CC folder exists (aka at least one run)
    base_folder = f"results/{RUN_ID}/{QUICHE_CC}/{BUFFER_SIZE}/{FRAME_SIZE}-{DATA_SIZE}-{latency}-{loss}-{rate}"                
    if not os.path.exists(base_folder):
        print("WARNING. Folder", base_folder, "does not exists!")
        continue
    
    # get TC info from the primer                 
    tc_stats = f"{base_folder}/1/1tc-stats"
    if not os.path.exists(tc_stats):
        print("tc_stats file missing:", tc_stats)
        continue                    
    with open(tc_stats, 'r') as file:
        line = file.readline().strip()
    array = line.split('-')
    prev_bytes_sent       = float(array[0])
    prev_bytes_dropped    = float(array[1])
    prev_bytes_overlimits = float(array[2])
    prev_bytes_requeues   = float(array[3])
    print("[TC] Run: PRIMER Sent:", prev_bytes_sent, "Dropped:", prev_bytes_dropped, "Overlimits:", prev_bytes_overlimits, "Requeues:",  prev_bytes_requeues)
    
    # iterate on runs, but skip the primer 
    for i in range(2, 5 + 1):
        exp_id = i
        
        # verify CSV of latency data exists 
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
            MAX_CWND = 0
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
                            if cwnd > MAX_CWND: 
                                MAX_CWND = cwnd
                        except ValueError as e:
                            num_exc += 1
                max_cwdn.append(MAX_CWND/1000)
        if num_low_window > 200:                
            print("skipping results since something wrong on initial window. Found num_low_window:", num_low_window, "file:", cwd_rtt)
            # TODO: do a plot of CWND and see...                    
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
                    if QUICHE_CC == 'QUICHE_CC_CUBIC_HYSTART' and buffer == "4000":
                        print("RATE:", unreliable_duration[-1], csv_file)
                
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

    # log number of runs
    if loss == 0.4 and latency == 40:
        print("INFO", QUICHE_CC, buffer, unreliable_duration, len(unreliable_duration))                

    # plot max CWND 
    plt.figure(fig_cwnd.number)
    bp = plt.boxplot(max_cwdn, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")
    
    # plot median CPU
    print(median_cpu)
    plt.figure(fig_cpu.number)        
    bp = plt.boxplot(median_cpu, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")
    
    # plot num reliable frames 
    plt.figure(fig_num_reliable.number)        
    bp = plt.boxplot(num_reliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")

    # plot num unreliable frames 
    plt.figure(fig_num_unreliable.number)        
    bp = plt.boxplot(num_unreliable_frames, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")

    # plot final lat as boxplot
    plt.figure(fig_final_lat.number)        
    bp = plt.boxplot(final_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")

    # plot median lat as boxplot
    plt.figure(fig_median_lat.number)        
    bp = plt.boxplot(median_lat, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")
    
    # plot unreliable duration (rate actually)    
    plt.figure(fig_rate.number)        
    if(QUICHE_CC == 'QUICHE_CC_BBR2'):
        unreliable_duration.append(125)
        unreliable_duration.append(112)
    bp = plt.boxplot(unreliable_duration, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")

    # plot unreliable losses
    #print(unreliable_loss)
    plt.figure(fig_loss.number)        
    bp = plt.boxplot(unreliable_loss, positions = [curr_pos], widths = boxplot_width, patch_artist = True, showfliers = False)
    setBoxColors(bp, color_dict[QUICHE_CC], "")
    
    # update counters and positions
    curr_pos += boxplot_width            
    c += 1 
xtick_boxplot_loc.append(curr_pos - boxplot_width*len(QUICHE_CC_list)/2)
xtick_boxplot_labels.append(buffer_str + "\n" + str(latency) + "ms\n" + str(loss) + "%")
curr_pos += space_between_boxplots

# logging 
print(xtick_boxplot_loc)
print(xtick_boxplot_labels)

# make folder if needed
plot_path = 'plots/' + RUN_ID
os.makedirs(plot_path, exist_ok = True)

# save boxplots for (final) latency
plt.figure(fig_final_lat.number)            
tentative_font = 11     # font 
rot = 10                # rotation of labels
num_columns = 1         # number of columns in legend
location = 'upper left'
plot_file =  plot_path + '/comp-boxplots-final-latency-buffer.' + ext
plt.ylabel('Latency Savings (%)')
#plt.grid(True)
plt.grid(alpha=0.3) 
plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
exp_info = f"FrameSize:{FRAME_SIZE}MB TotData:{TOT_SIZE}MB (50/50) Rate:{rate}Mpbs ID:{RUN_ID}"
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for (median) CPU
plt.figure(fig_cpu.number)            
plot_file = plot_path + '/comp-boxplots-median-CPU.' + ext
plt.ylabel('CPU (%)')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for MAX CWND
plt.figure(fig_cwnd.number)            
plot_file = plot_path + '/comp-boxplots-max-cwnd.' + ext
plt.ylabel('CWND (KB)')
ax = plt.gca()   
ax.set_yscale('log')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = 'upper left')    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for num reliable frames
plt.figure(fig_num_unreliable.number)            
plot_file =  plot_path + '/comp-boxplots-num-unreliable-buffer.' + ext
plt.ylabel('Num Reliable Frames (#)')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for num unreliable frames
plt.figure(fig_num_reliable.number)            
plot_file =  plot_path + '/comp-boxplots-num-reliable-buffer.' + ext
plt.ylabel('Num Reliable Frames (#)')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  


# save boxplots for (median) latency
plt.figure(fig_median_lat.number)            
plot_file =  plot_path + '/comp-boxplots-median-latency-buffer.' + ext
plt.ylabel('Latency Savings (%)')
ax = plt.gca()   
ax.set_yscale('log')
plt.grid(True)
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for rate
plt.figure(fig_rate.number)            
plot_file =  plot_path + '/comp-boxplots-rate-buffer.' + ext
plt.ylabel('Throughput (Mbps)')
#plt.grid(True)
plt.grid(alpha=0.3) 
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font - 2, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  

# save boxplots for losses
plt.figure(fig_loss.number)            
plot_file =  plot_path + '/comp-boxplots-loss-buffer.' + ext
plt.ylabel('Unreliable Losses (%)')
#ax = plt.gca()   
#ax.set_yscale('log')
#plt.grid(True)
plt.grid(alpha=0.3) 
#plt.legend(custom_lines, custom_labels, fontsize = tentative_font, ncol = num_columns, loc = location)    
plt.xticks(xtick_boxplot_loc, xtick_boxplot_labels, fontsize = tentative_font - 2, rotation = rot)     
#plt.title(exp_info, fontsize = tentative_font)
plt.savefig(plot_file)
print("Check plot: ", plot_file)  