#!/bin/bash

#t_start=`date +%s`
#cat ./results/1717776512/QUICHE_CC_BBR/2000000/1-100-10-0-100/4/4qlog_server.qlog | grep "congestion_window" | awk -F "," '{for (i = 1; i <= NF; i++) {if ($i ~ /congestion_window/) {cwd=$i; gsub(/[^0-9.]/, "", cwd)} if ( $i ~ /smoothed_rtt/){srtt=$i; gsub(/[^0-9.]/, "", srtt) }} time=$1; gsub(/[^0-9.]/, "", time); print time, cwd, srtt}' > cwd
#t_end=`date +%s`
#let "t_p = t_end - t_start"
#echo "TimePassed: ${t_p}"


for((exp_id=1; exp_id<=5; exp_id++))
do 
    res_folder="./results/1717776512/QUICHE_CC_BBR/2000000/1-100-10-0-100/${exp_id}/"
    cat ${res_folder}"/${exp_id}qlog_server.qlog" | grep "congestion_window" | awk -F "," '{for (i = 1; i <= NF; i++) {if ($i ~ /congestion_window/) {cwd=$i; gsub(/[^0-9.]/, "", cwd)} if ( $i ~ /smoothed_rtt/){srtt=$i; gsub(/[^0-9.]/, "", srtt) }} time=$1; gsub(/[^0-9.]/, "", time); print time, cwd, srtt}' > ${res_folder}"/${exp_id}cwd_rtt"
done