#!/bin/bash

# - publish each topic on new uav
# - $1 original uav name (e.g. uav45)
# - $2 new uav name (e.g. uav40)

command_name=$0
orig_uav=$1
new_uav=$2
merg_uav=$3
map_num=$4
rosbag_path=$5

topics=$(echo "$(rosbag info $rosbag_path)" | grep -Eo ".*${orig_uav}[^ ]*")
new_topics=$(echo "${topics}" | sed "s/${orig_uav}/${new_uav}/g")
renames=$(awk 'FNR==NR{topics[FNR]=$1; next} {printf "%s:=%s\n",topics[FNR],$1}' <(echo "${topics}") <(echo "${new_topics}"))

# don't send map to itself
if [ ${new_uav} != ${merg_uav} ]; then
  renames=$(sed '/map_throttle/d' <(echo "${renames}"))
  renames=$(sed '/odom_main/d' <(echo "${renames}"))
  rosbag play $rosbag_path /${orig_uav}/hector_mapping/map_throttle:=/${merg_uav}/hector_mapping/received_map${map_num} ${renames} /${orig_uav}/odometry/odom_main:=/${merg_uav}/odometry/odom_main_rec${map_num} ${renames} -l -s 138 -u 10
else
  rosbag play $rosbag_path ${renames} -l
fi
