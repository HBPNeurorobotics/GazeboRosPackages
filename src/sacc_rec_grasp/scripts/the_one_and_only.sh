#!/bin/bash

SESSIONNAME="sacc_rec_grasp"

spinnaker=false
inject_error=false
sim=false

while test $# -gt 0
do
    case "$1" in
        --spinnaker)
            spinnaker=true
            ;;
        --inject_error)
            inject_error=true
            ;;
        --sim)
            sim=true
            ;;
        --*) echo "bad option $1"
            ;;
    esac
    shift
done

rosrun catmux create_session package://sacc_rec_grasp/config/catmux_session.yaml --tmux_config package://catmux/etc/tmux_default.conf --session_name $SESSIONNAME --overwrite sim=${sim},spinnaker=${spinnaker},inject_error=${inject_error}
