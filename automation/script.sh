#!/bin/bash
## NOTE:     script to manage Octavius experiments 
## AUTHOR:   Matteo Varvello <matteo.varvello@nokia.com>
## DATE:     06/03/2024
## Example:  ./script.sh -f 4 -d 100 -c QUICHE_CC_CUBIC -D 10 -l 3 -r 200 -e 1234
## Example:  ./script.sh --frame-size 4 --data-size 100 --quiche-cc QUICHE_CC_CUBIC --delay 10 --loss 3 --rate 200 --exp-id 1234

# helper to cleanup things. Ignore <<Error: Cannot delete qdisc with handle of zero.>> if nothing was set
function net_cleanup(){
    myprint "Net emulation cleanup. Ignore <<Error: Cannot delete qdisc with handle of zero.>> if nothing was set"
    command="sudo tc qdisc del dev ${server_iface} root"  # cleanup prev rules
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}"
    command="sudo tc qdisc del dev ${client_iface} root" # cleanup prev rules
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "${command}"
}

# helper to stop potentially pending previous experiments
function stop_previous(){
    local_stop_all="false"
    if [ $1 == "after" -a ${stop_all} == "true" ]
    then
        myprint "Since we are done, do a full cleanup!"
        local_stop_all="true"
    fi 
    if [ $1 == "before" -a ${restart} == "true" ]
    then
        myprint "Since we are restarting and it is before script, forcing a full cleanup!"
        local_stop_all="true"
    fi 
    myprint "Stop potentially pending processes. STOP_ALL: ${local_stop_all}!"  
    command="./octree/stop.sh ${local_stop_all}"
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}"
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "${command}"
}

# helper to cleanup previous logs
function log_cleanup(){
    myprint "Cleanup potential previous log (server)..." 
    server_log="./octree/build/server-log-${exp_id}"
    server_qlog="./octree/build/qlog_server.qlog"
    cpu_log="./octree/build/cpu-log-${exp_id}"    
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "rm ${server_log} ${cpu_log} ${server_qlog}"
    
    # truncate server-1
    server_log_1="./octree/build/server-log-1"        
    myprint "Truncate ${server_log_1}"    
    #ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} ": > ${server_log_1}"
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "truncate -s 0 ${server_log_1}"
    
    myprint "Cleanup potential previous log (client)..."     
    client_log="./octree/build/client-log-${exp_id}"
    client_qlog="./octree/build/qlog_client.qlog"
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "rm ${client_log} ${client_qlog}"
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "rm ./octree/quic_benchmarks/data/${exp_id}*"
}


# Function to calculate the buffer size
function calculate_buffer_size() {
  local rate_mbit=$1
  local delay_ms=$2
  local margin_percent=$3

  # Convert rate from Mbit/s to bit/s
  local rate_bps=$(( rate_mbit * 1000000 ))

  # Convert delay from ms to seconds
  local delay_sec=$(echo "scale=6; $delay_ms / 1000" | bc)

  # Calculate BDP (Bandwidth-Delay Product) in bits
  local bdp_bits=$(echo "scale=0; $rate_bps * $delay_sec / 1" | bc)

  # Convert BDP from bits to bytes
  local bdp_bytes=$(echo "scale=0; $bdp_bits / 8" | bc)

  # Calculate buffer size with margin
  local margin_factor=$(echo "scale=2; 1 + $margin_percent / 100" | bc)
  local buffer_size_with_margin=$(echo "scale=0; $bdp_bytes * $margin_factor / 1" | bc)

  echo $buffer_size_with_margin
}


# simple function for logging
function myprint(){
    timestamp=`date +%s`
    val=$1
    if [ $# -eq  0 ]
    then
        return 
    fi
    echo -e "\033[32m[$0][$timestamp]\t${val}\033[0m"      
}

# verify QUICHE congestion control requested is supported
function verify_quiche_cc() {
    case "$1" in
        QUICHE_CC_RENO|QUICHE_CC_CUBIC|QUICHE_CC_BBR|QUICHE_CC_BBR2|QUICHE_CC_CUBIC_HYSTART)
            ;;
        *)
            echo "Error: Invalid QUICHE_CC option: $1. Supported options are: QUICHE_CC_RENO|QUICHE_CC_CUBIC|QUICHE_CC_BBR|QUICHE_CC_BBR2" >&2
            usage
            ;;
    esac
}

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT
function ctrl_c() {
    myprint "Trapped CTRL-C. Stop and cleanup"
    stop_all="true"
    stop_previous "after"
    net_cleanup
    exit -1 
}

# fixed parameters
ssh_server="charizard"
ssh_server_port="9999"
ssh_client="pikachu"
ssh_client_port="22"
server_IP="192.168.1.233"
server_PORT="12345"
key="id_rsa_local"
SRC_DIR="src/quiche_transport"
config_file="${SRC_DIR}/config.h"
server_file="${SRC_DIR}/server.cc"
client_file="${SRC_DIR}/client.cc"
server_qlog="./octree/build/qlog_server.qlog"
client_qlog="./octree/build/qlog_client.qlog"    
OCT_USER="haseeb"
server_iface="eth4"
client_iface="enp5s0"
sleep_time=10
MAX_VAL=250     # 255 is max number of frames (leaving some buffer)
#TIMEOUT=60     # max experiment duration (increase when rate drops due to losses and stuff...)
TIMEOUT=180
force_net_restart="false"    # flag to control if to force network emulation or not 

# potentially variable params (value below are "default")
FRAME_SIZE=4                 # size of a frame (MB)
DATA_SIZE=100                # volume of reliable and unrealiable traffic (MB)
QUICHE_CC="QUICHE_CC_CUBIC"  # congestion control for QUICHE
delay=20                     # full network delay
half_delay=$((delay / 2))    # half network delay, since added at both machine (ms)
loss=3                       # loss rate (%) - just at the server?
rate=200                     # throttling rate (Mbpbs)
exp_id="0"                   # experimenter identifier
stop_all="false"             # flag to control if to also stop server or not
to_update="false"            # flag to control if code should be updated or not 
restart="false"              # flag to control if server should be restarted or not 
BUFFER_SIZE=-1               # buffer size. If -1, use BDP to compute 
RUN_ID=`date +%s`            # unique run identifier 
use_qlog="false"             # flag to use qlog or not 
    
# Function to display usage information
usage() {
    echo "Usage: $0 [-f|--frame-size FRAME_SIZE] [-d|--data-size DATA_SIZE] [-c|--quiche-cc QUICHE_CC] [-D|--delay DELAY] [-l|--loss LOSS] [-r|--rate RATE] [-e|--exp-id EXP_ID] [-a|--all] [-u|--update] [-R|--restart] [-b|--buffer-size BUFFER_SIZE] [-q|--qlog]" 1>&2
    exit 1
}

# Parse command-line options
while [[ $# -gt 0 ]]; do
    case "$1" in
        -q|--qlog)
            use_qlog="true"
            ;;
        --ID)
            RUN_ID=$2
            shift
            ;;
        -R|--restart)
            restart="true"
            ;;
        -u|--update)
            to_update="true"
            ;;
        -a|--all)
            stop_all="true"
            ;;
        -b|--buffer-size)
            let "BUFFER_SIZE=$2*1000" #KB
            shift
            ;;
        -f|--frame-size)
            FRAME_SIZE=$2
            shift
            ;;
        -d|--data-size)
            DATA_SIZE=$2
            shift
            ;;
        -c|--quiche-cc)
            verify_quiche_cc $2
            QUICHE_CC=$2
            shift
            ;;
        -D|--delay)
            delay=$2
            half_delay=$((delay / 2))
            shift
            ;;
        -l|--loss)
            loss=$2
            shift
            ;;
        -r|--rate)
            rate=$2
            shift
            ;;
        -e|--exp-id)
            exp_id=$2
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            ;;
    esac
    shift
done

# local folder organization
if [ ${BUFFER_SIZE} != -1 ]
then
    res_folder="./results/${RUN_ID}/${QUICHE_CC}/${BUFFER_SIZE}/${FRAME_SIZE}-${DATA_SIZE}-${delay}-${loss}-${rate}/${exp_id}/"
else 
    res_folder="./results/${RUN_ID}/${QUICHE_CC}/${FRAME_SIZE}-${DATA_SIZE}-${delay}-${loss}-${rate}/${exp_id}/"
fi 
if [ -d ${res_folder} ]
then 
    rm -rf ${res_folder}
fi 
mkdir -p ${res_folder}

# Display updated parameters (optional)
myprint "FRAME_SIZE:${FRAME_SIZE}MB DATA_SIZE:${DATA_SIZE}MB QUICHE_CC:${QUICHE_CC} HALF_DELAY:${half_delay}ms (FULL:${delay}ms) LOSS:${loss}% RATE:${rate}Mbps EXP_ID:${exp_id} RESTART:${restart} TO_UPDATE:${to_update} STOP_ALL:${stop_all} RES_FOLDER:${res_folder} USE_QLOG:${use_qlog}"

# Check if loss is not greater than 0.8
if (( $(echo "$loss > 1.5" | bc -l) )); then
   myprint "ERROR Loss parameter cannot be more than 1.5 -- this will trigger a timeout?"
   exit -1
fi

# cleanup network emulation at client and server 
if [ ${restart} == true -o ${force_net_restart} == true ]
then
    net_cleanup
fi 

# cleanup any pending process
stop_previous "before"

# cleanup previous logs with same id
log_cleanup

# check for hystart
use_hystart="false"
SHORT_QUICHE_CC=${QUICHE_CC}
if [[ "$QUICHE_CC" == *"HYSTART"* ]]
then 
    use_hystart="true"
    SHORT_QUICHE_CC=${QUICHE_CC//_HYSTART/}
    myprint "Hystart was requested (only cubic allowed)"
fi 

# update config file at client and server
if [ ${to_update} == "true" ]
then 
    myprint "Recompiling both server and client!"

    # Ensure that (RELIABLE_DATA_SIZE + UNRELIABLE_DATA_SIZE)/FRAME_SIZE < 255    
    myprint "Updating config file. DATA_SIZE:${DATA_SIZE} FRAME_SIZE:${FRAME_SIZE} QUICHE_CC:${QUICHE_CC}"
    x=$(echo "scale=2; (2 * $DATA_SIZE) / $FRAME_SIZE" | bc)
    x_int=$(printf "%.0f" $x)
    if [ ${x_int} -gt ${MAX_VAL} ]
    then 
        myprint "ERROR, too many frames: ${x_int} Violated (RELIABLE_DATA_SIZE + UNRELIABLE_DATA_SIZE)/FRAME_SIZE < 255"
        exit -1
    fi 
    
    # restore original config file and modify with target value (MACOS only, fail on Linux)
    cp ${SRC_DIR}/config-original.h ${config_file}
    sed -i '' "s|100 \* 1024 \* 1024|${DATA_SIZE} * 1024 * 1024|g" ${config_file}
    sed -i '' "s|4 \* 1024 \* 1024|${FRAME_SIZE} * 1024 * 1024|g" ${config_file}

    # restore original server file and modify with target CC and qlog or not (MACOS only, fail on Linux)
    cp ${SRC_DIR}/server-original.cc ${server_file}    
    sed -i '' "s/quiche_config_set_cc_algorithm(config, QUICHE_CC_CUBIC)/quiche_config_set_cc_algorithm(config, ${SHORT_QUICHE_CC})/" ${server_file}    
    if [ ${use_hystart} == "true" ]
    then 
        sed -i '' "s/quiche_config_enable_hystart(config, false);/quiche_config_enable_hystart(config, true);/" ${server_file}    
    fi 
    if [ ${use_qlog} == "true" ] 
    then
        sed -i '' 's|bool use_qlog = false;|bool use_qlog = true;|' ${server_file}    
    fi

    # restore original client file and modify with target CC and qlog or not (MACOS only, fail on Linux)
    cp ${SRC_DIR}/client-original.cc ${client_file}    
    sed -i '' "s/quiche_config_set_cc_algorithm(config, QUICHE_CC_CUBIC)/quiche_config_set_cc_algorithm(config, ${SHORT_QUICHE_CC})/" ${client_file}    
    if [ ${use_hystart} == "true" ]
    then 
        sed -i '' "s/quiche_config_enable_hystart(config, false);/quiche_config_enable_hystart(config, true);/" ${client_file}    
    fi 
    #if [ ${use_qlog} == "true" ] 
    #then
    #    sed -i '' 's|bool use_qlog = false;|bool use_qlog = true;|' ${client_file}    
    #fi

    # update files at both server and client
    myprint "Copying new config file to both client and server..."
    scp -i ${key} -P ${ssh_server_port} ${config_file} ${client_file} ${server_file} ${OCT_USER}@${ssh_server}:"./octree/src/quiche_transport"
    scp -i ${key} -P ${ssh_client_port} ${config_file} ${client_file} ${server_file} ${OCT_USER}@${ssh_client}:"./octree/src/quiche_transport"

    # compile code at server and client
    myprint "Remote compilation..."
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "cd octree/build && make quiche_server"
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "cd octree/build && make quiche_client"
fi 


# network  emulation 
margin_percent=25  # Margin percentage
buffer_size=$(calculate_buffer_size ${rate} ${delay} ${margin_percent})
limit_size=$(echo "scale=0; $buffer_size * 1.5 / 1" | bc)
if [ ${restart} == true -o ${force_net_restart} == true ]
then
    # network emulation at server (charizard)
    myprint "Network emulation at the server (${ssh_server}). Delay:${delay}ms (Half:${half_delay}ms) Loss:${loss}% Rate:${rate}Mbps Buffer:${buffer_size}Bytes Limit:${limit_size}Bytes"
    if [ ${BUFFER_SIZE} != -1 ]
    then 
        myprint "Replacing buffer size with: ${BUFFER_SIZE}"
        buffer_size=${BUFFER_SIZE}
        limit_size=${BUFFER_SIZE}
    fi 

    # set network delay and losses
    #loss_burst=30  #corrupt_burst=10
    command="sudo tc qdisc add dev ${server_iface} root handle 1:0 netem delay ${half_delay}ms loss ${loss}% limit 3000"
    #command="sudo tc qdisc add dev ${server_iface} root handle 1:0 netem delay ${half_delay}ms loss ${loss}% ${loss_burst}% corrupt ${corrupt_burst}%"
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}"
    
    # adjust rate and buffer sizes
    command="sudo tc qdisc add dev ${server_iface} parent 1:1 handle 10: tbf rate ${rate}Mbit buffer ${buffer_size} limit ${limit_size}" 
    #command="sudo tc qdisc add dev ${server_iface} parent 1:1 handle 10: tbf rate ${rate}Mbit buffer 1Mb limit 1Mb" 
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}"

    # network emulation at client (pikachu)
    myprint "Network emulation at the client (${ssh_client})..."
    
    # set network delay and losses    
    command="sudo tc qdisc add dev ${client_iface} root handle 1:0 netem delay ${half_delay}ms loss 0% limit 3000"  
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "${command}"
    
    # adjust rate and buffer sizes    
    command="sudo tc qdisc add dev ${client_iface} parent 1:1 handle 10: tbf rate ${rate}Mbit buffer ${buffer_size} limit ${limit_size}"     
    #command="sudo tc qdisc add dev ${client_iface} parent 1:1 handle 10: tbf rate ${rate}Mbit buffer 1Mb limit 1Mb"   
    ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "${command}"

    # give some time to settle
    myprint "Done with net emulation. Allowing 5 seconds to settle..."
    sleep 5

fi 
## for not network emulation 
#command="sudo tc qdisc add dev ${server_iface} root fq maxrate 2gbit"
#    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}"

# start the server
## testing always restarting
if [ ${restart} == true ]
then
    server_log="server-log-${exp_id}"
    myprint "Starting server. Check log: ${server_log}"
    ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "cd octree/build && ./quiche_server ${server_IP} ${server_PORT} > ${server_log} 2>&1 &" &
fi 

# allow server warmup or things to calm down...
myprint "Sleep ${sleep_time} to warmup server or in between experiments (when no server warmup)!"
sleep ${sleep_time}

# start monitoring CPU usage at the server
cpu_log="cpu-log-${exp_id}"
myprint "Starting CPU monitoring at server. Check log: ${cpu_log}"
ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "cd octree/quic_benchmarks/cpu && ./monitor.sh ${server_PORT} a > ${cpu_log} 2>&1 &" &

# start the client 
client_log="client-log-${exp_id}"
myprint "Starting client. Check log: ${client_log}"
ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "cd octree/build && ./quiche_client ${server_IP} ${server_PORT} ${exp_id} > ${client_log} 2>&1 &" &

# wait for experiment to be over
sleep 2
myprint "Start monitoring for experiment to be over..."
t_start=`date +%s`
was_timeout="false"
while true 
do 
    command="ps aux | grep "quiche_client" | grep -v \"grep\" | grep -v \"bash\" | wc -l"
    ans=`ssh -i ${key} -p ${ssh_client_port} ${OCT_USER}@${ssh_client} "${command}"`
    myprint "Active clients: ${ans}"
    if [ ${ans} -gt 0 ]
    then 
        t_end=`date +%s`
        let "t_passed = t_end - t_start"
        if [ ${t_passed} -gt ${TIMEOUT} ]
        then 
            myprint "ERROR! TIMEOUT"
            was_timeout="true"
            break
        fi 
        sleep 5 
    else 
        break
    fi 
done

# collect TC stats
command="tc -s qdisc show dev ${server_iface}"
ssh -i ${key} -p ${ssh_server_port} ${OCT_USER}@${ssh_server} "${command}" > "${res_folder}/${exp_id}tc-stats-full"
cat "${res_folder}/${exp_id}tc-stats-full" | grep "Sent" | tail -n 1 | awk '{gsub(/,|\)/,""); print $2"-"$7"-"$9"-"$11}' > "${res_folder}/${exp_id}tc-stats"

# cleanup any pending process
stop_previous "after"

# cleanup network emulation at client and server (unless ongoing experiment)
if [ ${stop_all} == "true" ]
then 
    net_cleanup
fi 

# collect logs
if [ ${was_timeout} == "false" ] 
then 
    myprint "Collecting server logs..."
    server_log_1="./octree/build/server-log-1"    
    scp -i ${key} -P ${ssh_server_port} ${OCT_USER}@${ssh_server}:"${server_log_1}" "${res_folder}/server-log-${exp_id}"
    scp -i ${key} -P ${ssh_server_port} ${OCT_USER}@${ssh_server}:"./octree/quic_benchmarks/cpu/${cpu_log}" "${res_folder}/${exp_id}cpu-log"
    if [ ${use_qlog} == "true" ] 
    then
        scp -i ${key} -P ${ssh_server_port} ${OCT_USER}@${ssh_server}:"${server_qlog}" ${res_folder}"/${exp_id}qlog_server.qlog"
        cat ${res_folder}"/${exp_id}qlog_server.qlog" | grep "congestion_window" | awk -F "," '{for (i = 1; i <= NF; i++) {if ($i ~ /congestion_window/) {cwd=$i; gsub(/[^0-9.]/, "", cwd)} if ( $i ~ /smoothed_rtt/){srtt=$i; gsub(/[^0-9.]/, "", srtt) }} time=$1; gsub(/[^0-9.]/, "", time); print time, cwd, srtt}' > ${res_folder}"/${exp_id}cwd_rtt"
    fi

    myprint "Collecting client logs..."    
    scp -i ${key} -P ${ssh_client_port} ${OCT_USER}@${ssh_client}:"./octree/build/${client_log}" ${res_folder}
    scp -i ${key} -P ${ssh_client_port} ${OCT_USER}@${ssh_client}:"./octree/quic_benchmarks/data/${exp_id}*" ${res_folder}    
    #if [ ${use_qlog} == "true" ] # skip to save time, unused anyway...
    #then
    #    scp -i ${key} -P ${ssh_client_port} ${OCT_USER}@${ssh_client}:"${client_qlog}" ${res_folder}"/${exp_id}qlog_client.qlog"
    #fi 

    # analysis/plotting
    myprint "/usr/bin/python3 analyze.py ${QUICHE_CC} ${delay} ${loss} ${rate} ${exp_id} ${FRAME_SIZE} ${DATA_SIZE} ${BUFFER_SIZE} ${RUN_ID}"
    /usr/bin/python3 analyze.py ${QUICHE_CC} ${delay} ${loss} ${rate} ${exp_id} ${FRAME_SIZE} ${DATA_SIZE} ${BUFFER_SIZE} ${RUN_ID}
else
    myprint "Skipping collecting logs due to timeout"
fi 

# All done
myprint "All done!"