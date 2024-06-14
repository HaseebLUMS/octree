#!/bin/bash
## NOTE:     master script to control paper experiments
## AUTHOR:   Matteo Varvello <matteo.varvello@nokia.com>
## DATE:     06/03/2024

# simple function for logging
function myprint(){
    timestamp=`date +%s`
    val=$1
    if [ $# -eq  0 ]
    then
        return 
    fi
    echo -e "\033[31m[$0][$timestamp]\t${val}\033[0m"      
}


# trap ctrl-c and call ctrl_c()
#trap ctrl_c INT
#function ctrl_c() {
#    myprint "Trapped CTRL-C. Stop and cleanup"    
#    ## TODO
#    exit -1 
#}

# default parameters
FRAME_SIZE=1            # size of a frame (MB)
DATA_SIZE=100           # volume of reliable and unrealiable traffic (MB)
#rate=200               # throttling rate (Mbpbs)
rate=250
#rate=300              
exp_id=1                # increasing exp identifier
NUM_REPS=5             # number of repetitions  (at least 2, since need a primer)
use_qlog="false"        # flag to control if storing qlogs or not
#use_qlog="false"       # flag to control if storing qlogs or not
#RUN_ID="1717687086"    # use this run identifier to replace results 
#RUN_ID="1717696476"
#RUN_ID="1717700544"    # full, 10ms (make sense)
#RUN_ID="1717709559"    # full, 40 ms (wide boxplots)
#RUN_ID="1717776512"    # full, 10ms non losses and 100Mbps (had to redo BBR, added skip on CWND - requires QLOGS)
#RUN_ID="1717791985"    # full, 10ms non losses and 200Mbps (CWND check) 
#RUN_ID="1718049862"    # testing 300Mbps...
RUN_ID="1718059665"     # testing 250Mbps...

# variable parameters
latencies=("10")
losses=("0")
#quiche_cc_options=("QUICHE_CC_RENO" "QUICHE_CC_CUBIC" "QUICHE_CC_CUBIC_HYSTART" "QUICHE_CC_BBR" "QUICHE_CC_BBR2")
#quiche_cc_options=("QUICHE_CC_CUBIC" "QUICHE_CC_CUBIC_HYSTART" "QUICHE_CC_BBR" "QUICHE_CC_BBR2")
quiche_cc_options=("QUICHE_CC_BBR")
#buffer_sizes=(100 200 500 1000 2000 4000) #KB
buffer_sizes=(100) #KB

# log plotting command for early plotting
QUICHE_CC_STR="${quiche_cc_options[0]}"
for ((i = 1; i < ${#quiche_cc_options[@]}; i++)); do
    QUICHE_CC_STR+=",${quiche_cc_options[i]}"
done
delay_str="${latencies[0]}"
for ((i = 1; i < ${#latencies[@]}; i++)); do
    delay_str+=",${latencies[i]}"
done
loss_str="${losses[0]}"
for ((i = 1; i < ${#losses[@]}; i++)); do
    loss_str+=",${losses[i]}"
done
BUFFER_str="${buffer_sizes[0]}"
for ((i = 1; i < ${#buffer_sizes[@]}; i++)); do
    BUFFER_str+=",${buffer_sizes[i]}"
done
myprint "/usr/bin/python3 master-buffer-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${BUFFER_str} ${RUN_ID}"

# start a time to measure duration 
start_time=`date +%s`

# Iterate over QUICHE_CC options
for quiche_cc in "${quiche_cc_options[@]}"; do

    # flag down to recompile
    to_recompile="true"

    # Iterate over buffer_sizes
    for buffer in "${buffer_sizes[@]}"; do
        # Iterate over latencies
        for latency in "${latencies[@]}"; do
            # Iterate over losses
            for loss in "${losses[@]}"; do
                # Iterate over target NUM_REPS
                for ((i=1; i<=NUM_REPS; i++)); do

                    ### temporarily skipping run 2 and 3 do to a mistake...
                    #if [ ${i} -eq 2 -o ${i} -eq 3 ]
                    #then 
                    #    continue
                    #fi 
                    ### temporarily skipping run 2 and 3 do to a mistake...
                    
                    exp_id=${i}
                    OPT="-f ${FRAME_SIZE} -d ${DATA_SIZE} -c ${quiche_cc} -D ${latency} -l ${loss} -r ${rate} -e ${exp_id} -b ${buffer} --ID ${RUN_ID}"
                    if [ ${use_qlog} == "true" ] 
                    then 
                        OPT=$OPT" -q"
                    fi 

                    # recompile if a new CC
                    if [ ${to_recompile} == "true" ] 
                    then
                        OPT=$OPT" -u"  #-u) recompile code
                        to_recompile="false" 
                    fi 

                    # special options for the primer
                    if [ $i -eq 1 ]
                    then 
                        #OPT=$OPT" -u -R"  #-u) recompile code, -r) restart
                        OPT=$OPT" -R"      #-r) restart                        
                    fi 

                    # final option to stop things 
                    if [ $i -eq ${NUM_REPS} ]
                    then 
                        OPT=$OPT" -a"    # -a) kill everything in exit
                    fi 
                    myprint "./script.sh ${OPT}"
                    ./script.sh ${OPT}                
                done
            done
        done
    done
done

# derive duration 
end_time=`date +%s`
let "t_passed = end_time - start_time"
myprint "Duration ${t_passed}"

# do plotting
myprint "/usr/bin/python3 master-buffer-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${BUFFER_str} ${RUN_ID}"
/usr/bin/python3 master-buffer-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${BUFFER_str} ${RUN_ID}