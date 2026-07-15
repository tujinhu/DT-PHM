#!/bin/bash
# run multiple instances of the 'px4' binary, but w/o starting the simulator.
# It assumes px4 is already built, with 'make px4_sitl_default'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example jmavsim can be run like this:
#./Tools/jmavsim_run.sh -p 4561 -l


RFLY_SIL_UDP_RECEIVE=16540
RFLY_SIL_UDP_SEND=17540
RFLY_GCS_UDP_SEND=18570

sitl_num=2
[ -n "$1" ] && sitl_num="$1"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/px4_sitl_default

# echo "killing running instances"
# pkill -x px4 || true

# sleep 1


PX4SiltFrame=iris
[ -n "$3" ] && [ "$3" != "0" ] && [ "$3" != "1" ] && PX4SiltFrame="$3"
export PX4_SIM_MODEL=$PX4SiltFrame

airframe_path=${build_path}/etc/init.d/airframes
airframe_path_sitl=${build_path}/etc/init.d-posix/airframes

FileN0=$(find ${airframe_path_sitl} -name "*[0-9]_${PX4SiltFrame}" | head -n 1 | sed -e 's|/.*/||')
if [ -z "$FileN0" ]; then
    echo "No Airframe Name: ${PX4SiltFrame} in init.d-posix Folder"
    FileN=$(find ${airframe_path} -name "*[0-9]_${PX4SiltFrame}" | head -n 1 | sed -e 's|/.*/||')
    if [ -z "$FileN" ]; then
        echo "Wrong Airframe Name: ${PX4SiltFrame}"
        exit 1
    else
        echo "Using Airframe File: ${FileN}"
    fi

    if [ ! -f ${airframe_path_sitl}/${FileN} ]; then
        echo "Copy Airframe file ${FileN}"
        \cp -rf ${airframe_path}/${FileN} ${airframe_path_sitl}/${FileN}
    fi

else
    echo "Using Airframe File: ${FileN0}"
fi


rcs_script_path=${build_path}/etc/init.d-posix
rcs_romfs_path=${src_path}/ROMFS/px4fmu_common/init.d-posix


# if [ ! -f $SCRIPT_DIR/rcS ]; then
#     \cp -rf $rcs_script_path/rcS $SCRIPT_DIR/rcS
#     \cp -rf $rcs_script_path/px4-rc.mavlink $SCRIPT_DIR/px4-rc.mavlink
#     \cp -rf $rcs_script_path/px4-rc.simulator $SCRIPT_DIR/px4-rc.simulator
# else
#     \cp -rf $SCRIPT_DIR/rcS $rcs_script_path/rcS
#     \cp -rf $SCRIPT_DIR/px4-rc.mavlink $rcs_script_path/px4-rc.mavlink
#     \cp -rf $SCRIPT_DIR/px4-rc.simulator $rcs_script_path/px4-rc.simulator
# fi

RFLY_SIL_UDP_RECEIVE=$(($RFLY_SIL_UDP_RECEIVE-1))
RFLY_SIL_UDP_SEND=$(($RFLY_SIL_UDP_SEND-1))
RFLY_GCS_UDP_SEND=$(($RFLY_GCS_UDP_SEND-1))

n=1
[ -n "$2" ]  &&  n="$2"
sitl_num=$(($sitl_num + $n))
#make px4_sitl_default

if [ $sitl_num -le 9 ]; then
    cat ${rcs_romfs_path}/px4-rc.simulator | sed s/4560/4559/ > $rcs_script_path/px4-rc.simulator

    cat ${rcs_romfs_path}/rcS | sed s/'px4_instance+1'/'px4_instance'/ > $rcs_script_path/rcS

    cat ${rcs_romfs_path}/px4-rc.mavlink | sed s/14540/${RFLY_SIL_UDP_RECEIVE}/ | \
        sed s/14580/${RFLY_SIL_UDP_SEND}/ | \
        sed s/18570/${RFLY_GCS_UDP_SEND}/ | \
        sed s/'mavlink start -x -u $udp_offboard_port_local'/'# mavlink start -x -u $udp_offboard_port_local'/ | \
        sed s/'mavlink start -x -u $udp_gcs_port_local -r 4000000'/'mavlink start -x -u $udp_offboard_port_local -r 4000000 -o $udp_offboard_port_remote'/ | \
        sed s/'-u $udp_gcs_port_local'/'-u $udp_offboard_port_local'/ | \
        sed s/'\[ $px4_instance -gt 9 \]'/'#\[ $px4_instance -gt 9 \]'/ > $rcs_script_path/px4-rc.mavlink
else

    cat ${rcs_romfs_path}/px4-rc.simulator | sed s/4560/4559/ > $rcs_script_path/px4-rc.simulator

    cat ${rcs_romfs_path}/rcS | sed s/'px4_instance+1'/'px4_instance'/ > $rcs_script_path/rcS

    cat ${rcs_romfs_path}/px4-rc.mavlink | sed s/14540/${RFLY_SIL_UDP_RECEIVE}/ | \
        sed s/14580/${RFLY_SIL_UDP_SEND}/ | \
        sed s/18570/${RFLY_GCS_UDP_SEND}/ | \
        sed s/'mavlink start -x -u $udp_offboard_port_local'/'# mavlink start -x -u $udp_offboard_port_local'/ | \
        sed s/'mavlink start -x -u $udp_gcs_port_local -r 4000000'/'mavlink start -x -u $udp_offboard_port_local -r 4000000 -o $udp_offboard_port_remote'/ | \
        sed s/'mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED'/'# mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED'/ | \
        sed s/'LOCAL_POSITION_NED -u $udp_gcs_port_local'/'LOCAL_POSITION_NED -u $udp_offboard_port_local'/ | \
        sed s/'50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local'/'10 -s GLOBAL_POSITION_INT -u $udp_offboard_port_local'/ | \
        sed s/'ATTITUDE -u $udp_gcs_port_local'/'ATTITUDE -u $udp_offboard_port_local'/ | \
        sed s/'mavlink stream -r 50 -s ATTITUDE_QUATERNION'/'# mavlink stream -r 50 -s ATTITUDE_QUATERNION'/ | \
        sed s/'mavlink stream -r 50 -s ATTITUDE_TARGET'/'# mavlink stream -r 50 -s ATTITUDE_TARGET'/ | \
        sed s/'mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0'/'# mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0'/ | \
        sed s/'mavlink stream -r 20 -s RC_CHANNELS'/'# mavlink stream -r 20 -s RC_CHANNELS'/ | \
        sed s/'mavlink stream -r 10 -s OPTICAL_FLOW_RAD'/'# mavlink stream -r 10 -s OPTICAL_FLOW_RAD'/ | \
        sed s/'\[ $px4_instance -gt 9 \]'/'#\[ $px4_instance -gt 9 \]'/ > $rcs_script_path/px4-rc.mavlink

fi
sleep 1


while [ $n -lt $sitl_num ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
	sleep 2
	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	../bin/px4 -i $n -d "$build_path/etc" -s etc/init.d-posix/rcS >out.log 2>err.log &
	popd &>/dev/null

	n=$(($n + 1))
    sleep 1
done

sleep 5
echo Copying rcS files
\cp -rf ${rcs_romfs_path}/rcS $rcs_script_path/rcS
\cp -rf ${rcs_romfs_path}/px4-rc.mavlink $rcs_script_path/px4-rc.mavlink
\cp -rf ${rcs_romfs_path}/px4-rc.simulator $rcs_script_path/px4-rc.simulator

echo PX4 instances start finished
