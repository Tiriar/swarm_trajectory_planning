#!/bin/bash

SESSION_NAME=gazebo

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  SESSION_NAME="$(tmux display-message -p '#S')"
fi

# UNCOMMENT IF YOU WANT TO LOG SESSIONS 
#LOG_DIR=~/logs_tmux
#suffix=$(date +"%Y_%m_%d_%H_%M_%S")
#SUBLOG_DIR=$LOG_DIR"/logs_"$suffix
#
#mkdir $LOG_DIR > /dev/null 2> /dev/null
#mkdir $SUBLOG_DIR
#unlink $LOG_DIR/latest
#ln -s $SUBLOG_DIR $LOG_DIR/latest

# TMUX= tmux new-session -s $SESSION_NAME -d

# key bindings for subpanels
tmux bind -n M-Left select-pane -L
tmux bind -n M-Right select-pane -R
tmux bind -n M-Up select-pane -U
tmux bind -n M-Down select-pane -D

# remap Ctrl-b
tmux unbind C-b
tmux set -g prefix C-a
tmux bind C-a send-prefix

# define commands
# 'name' 'command'
input=(
 	"Gazebo" "roslaunch simulation simulation.launch
"
	"Spawn" "sleep 10; spawn 1 2 --enable-bluefox-camera --enable-mobius-camera --enable-rangefinder --run --delete
"
	"KILL_ALL" "tmux kill-session -t $SESSION_NAME"
)

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+10)) -n "${names[$i]}"
done

sleep 4

# UNCOMMENT IF YOU WANT TO LOG SESSIONS
#for ((i=0; i < ${#names[*]}; i++));
#do
#	tmux pipe-pane -t $SESSION_NAME:$(($i+10)) -o "ts | cat >> $SUBLOG_DIR/$(($i+10))_${names[$i]}.log"
#done
#
#sleep 1

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+10)) "${cmds[$i]}"
done

sleep 4

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear
