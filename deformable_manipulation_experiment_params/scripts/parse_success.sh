#!/usr/bin/env bash

test_id=voxnet_old_labels_10_trials

environments[0]=rope_hooks_simple
environments[1]=rope_hooks
environments[2]=engine_assembly
environments[3]=rope_hooks_multi
environments[4]=rope_hooks_simple_super_long_rope
environments[5]=rope_hooks_simple_short_rope

for exp in ${environments[@]}; do
    cd ~/Dropbox/catkin_ws/src/smmap/logs/${exp}/${test_id}/basic__normalized_lengths__raw_connected_components/
    data=`cat */smmap_output.log | grep "Total successful"`
    echo "${exp} ${data}"
done
