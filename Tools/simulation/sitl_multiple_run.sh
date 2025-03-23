#!/bin/bash
# run multiple instances of the 'tc' binary, but w/o starting the simulator.
# It assumes tc is already built, with 'make tc_sitl_default'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example jmavsim can be run like this:
#./Tools/simulation/jmavsim/jmavsim_run.sh -p 4561 -l

sitl_num=2
[ -n "$1" ] && sitl_num="$1"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../"

build_path=${src_path}/build/tc_sitl_default

echo "killing running instances"
pkill -x tc || true

sleep 1

export TC_SIM_MODEL=gazebo-classic_iris

n=0
while [ $n -lt $sitl_num ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	$build_path/bin/tc -i $n -d "$build_path/etc" >out.log 2>err.log &
	popd &>/dev/null

	n=$(($n + 1))
done
