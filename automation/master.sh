#!/bin/bash
## NOTE:     master script to control paper experiments
## AUTHOR:   Matteo Varvello <matteo.varvello@nokia.com>
## DATE:     06/03/2024


####################### TCP NOTES ######################
#1) BBR is designed to keep 2BDP of data in the network => we see this in buffer plot
####  For instance, the throughput drops to half from 0.1% to 0.4% of packet loss => https://www.thousandeyes.com/blog/a-very-simple-model-for-tcp-throughput


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

# default parameters
FRAME_SIZE=1         # size of a frame (MB)
DATA_SIZE=100        # volume of reliable and unrealiable traffic (MB)
rate=250             # throttling rate (Mbpbs)
exp_id=1             # increasing exp identifier
use_qlog="false"     # flag to control if storing qlogs or not
NUM_REPS=5           # number of repetitions  (at least 2, since need a primer)
#RUN_ID="1717687086" # use this run identifier to replace results 
#RUN_ID="1717708139"
#RUN_ID="1717720225" # last full run (BBR only one weird, investigating)
#RUN_ID="1717765232" # investigating weird BBR runs (with qlog)
#RUN_ID="1717770636" # investigating weird BBR runs (without qlog)
#################
#RUN_ID="1717894056"  # full run up to 10ms, expand with more values
#RUN_ID="1717933280"  # ongoing run as above, but lower rate
#RUN_ID="1718111451"  # testing at 250Mbps 
RUN_ID="1718214175"   # 250Mbps, bigger buffer just 40ms with 0.4% losses

# variable parameters
#latencies=("10" "20" "40")
#losses=("0.1" "0.2" "0.4")
#quiche_cc_options=("QUICHE_CC_RENO" "QUICHE_CC_CUBIC" "QUICHE_CC_CUBIC_HYSTART" "QUICHE_CC_BBR" "QUICHE_CC_BBR2")
latencies=("40")
losses=("0.4")
#quiche_cc_options=("QUICHE_CC_BBR" "QUICHE_CC_BBR2")
quiche_cc_options=("QUICHE_CC_CUBIC" "QUICHE_CC_CUBIC_HYSTART" "QUICHE_CC_BBR" "QUICHE_CC_BBR2")

#####################################
#latencies=("10")
#losses=("0")
#NUM_REPS=3
#quiche_cc_options=("QUICHE_CC_RENO" "QUICHE_CC_BBR")
#####################################

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

# testing with fixed buffer
buffer_size=2000
myprint "/usr/bin/python3 master-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${RUN_ID} ${buffer_size}"
myprint "WARNING: fixed buffer size at 1MB!!!"


# Iterate over QUICHE_CC options
for quiche_cc in "${quiche_cc_options[@]}"; do

    # flag down to recompile
    to_recompile="true"

    # Iterate over latencies
    for latency in "${latencies[@]}"; do
        # Iterate over losses
        for loss in "${losses[@]}"; do
            # Iterate over target NUM_REPS
            for ((i=1; i<=NUM_REPS; i++)); do
                exp_id=${i}
                OPT="-f ${FRAME_SIZE} -d ${DATA_SIZE} -c ${quiche_cc} -D ${latency} -l ${loss} -r ${rate} -e ${exp_id} --ID ${RUN_ID} -b ${buffer_size}"
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

# derive duration 
end_time=`date +%s`
let "t_passed = end_time - start_time"
myprint "Duration ${t_passed}"

# do plotting
myprint "/usr/bin/python3 master-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${RUN_ID} ${buffer_size}"
/usr/bin/python3 master-analyze.py ${QUICHE_CC_STR} ${delay_str} ${loss_str} ${rate} ${NUM_REPS} ${FRAME_SIZE} ${DATA_SIZE} ${RUN_ID} ${buffer_size}