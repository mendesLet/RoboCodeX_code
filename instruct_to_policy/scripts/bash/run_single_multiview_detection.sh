# bash function to collect multiview detection dataset from a single gazebo world:
# - roslaunch a gazebo world and wait for 5 seconds to let the world load
# - rosrun the generate_multiview_detection.py node to collect the dataset
# - kill the gazebo world once the generate_multiview_detection.py node is done


kill_descendant_processes() {
    local pid="$1"
    local and_self="${2:-false}"
    if children="$(pgrep -P "$pid")"; then
        for child in $children; do
            kill_descendant_processes "$child" true
        done
    fi
    if [[ "$and_self" == true ]]; then
        kill -9 "$pid"
    fi
}

# NOTE: this script will kill the timeout functions itself 
# self_timeout() {
#     local timeout="$1"
#     local pid="$$"
    
#     (sleep "$timeout"; echo "Timeouting myself in $timeout seconds (PID $pid)"; kill_descendant_processes "$pid"; kill -9 "$pid") &
# }

# run the collect_multiview_dataset function in a subshell
collect_multiview_dataset() {
  world_path=$1
  out_dir=$2
  echo "Collecting multiview dataset for $world_path"

  # Launch the Gazebo world
  roslaunch instruct_to_policy run_panda_moveit_gazebo.launch \
    world:=$world_path \
    moveit_rviz:=false \
    gazebo_gui:=false &

  # Wait for the Gazebo world to load
  sleep 15

  # Run the generate_multiview_detection.py node to collect the dataset
  world_file=$(basename $world_path)
  world_name="${world_file%.*}"
  timeout 30 python scripts/data/generate_multiview_detection.py --world_name $world_name --output_dir $out_dir
  sleep 5
}

world_path=$1
out_dir=$2

collect_multiview_dataset $world_path $out_dir

# clean up any remaining gazebo instances
kill_descendant_processes $$