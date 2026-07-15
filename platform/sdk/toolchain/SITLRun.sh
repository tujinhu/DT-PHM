#!/usr/bin/env bash

# The text starting with '#' is annotation. The following options correspond to
# CopterSim/RflySim startup options. This script is based on SITLRun_.sh, with
# RFV runtime markers added so Python configs can set vehicle count/index.

# BEGIN RFV_RUNTIME_CONFIG
export RFV_VEHICLE_NUM=1
export RFV_START_INDEX=1
# END RFV_RUNTIME_CONFIG

SHPath="$(cd "$(dirname "$0")"; pwd)"
USRPath="${SHPath#/home/}"
USRPath="${USRPath%%/*}"

if [ -z "${PSP_PATH:-}" ]; then
    PSP_PATH="/home/${USRPath}/PX4PSP"
fi
if [ -z "${PSP_PATH_LINUX:-}" ]; then
    PSP_PATH_LINUX="${PSP_PATH}"
fi

CMDPre=""
if [ "$UID" -eq "0" ] 2>/dev/null; then
    CMDPre="su ${USRPath} -c"
    echo "Run as root"
fi

# Start index of vehicle number (should be larger than 0).
START_INDEX="${RFV_START_INDEX:-1}"

# Total vehicle number to auto arrange position.
# TOTOAL_COPTER=8

# Set the vehicleType/ClassID of vehicle 3D display in RflySim3D.
CLASS_3D_ID="${RFV_CLASS_3D_ID:-1}"

# Set use DLL/SO model name or not, use number index or name string.
DLLModel="${RFV_DLL_MODEL:-MulticopterNoCtrl}"

if [[ "${DLLModel}" =~ ^-?[0-9]+$ ]]; then
    echo "Use CopterSim internal model"
else
    echo "Copy the latest dll/so file to CopterSim folder"
    cp -rf "${SHPath}/${DLLModel}.so" "${PSP_PATH}/CopterSim/external/model/${DLLModel}.so"
fi

# Set the simulation mode on CopterSim. SimMode=2 equals PX4_SITL_RFLY.
SimMode="${RFV_SIM_MODE:-2}"

# Set the vehicle model/airframe of PX4 SITL simulation.
PX4SitlFrame="${PX4_SITL_FRAME:-iris}"

# Set the map, use index or name of the map on CopterSim.
UE4_MAP="${RFV_UE4_MAP:-VisionRing}"

# Set the origin x/y position and yaw angle at the map.
ORIGIN_POS_X="${RFV_ORIGIN_POS_X:-0}"
ORIGIN_POS_Y="${RFV_ORIGIN_POS_Y:-0}"
ORIGIN_YAW="${RFV_ORIGIN_YAW:-0}"

# Set the interval between two vehicles, unit: m.
VEHICLE_INTERVAL="${RFV_VEHICLE_INTERVAL:-2}"

# Set broadcast to other computer.
IS_BROADCAST="${RFV_IS_BROADCAST:-0}"

# Set UDP data mode: 0 UDP_FULL, 1 UDP_Simple, 2 Mavlink_Full, 3 Mavlink_simple.
UDPSIMMODE="${RFV_UDPSIMMODE:-0}"

MAX_VEHICLE=50

if [ -n "${RFV_VEHICLE_NUM:-}" ]; then
    VehicleNum="${RFV_VEHICLE_NUM}"
else
    while true; do
        read -r -p "Please input UAV swarm number: " VehicleNum
        if [[ "${VehicleNum}" =~ ^[0-9]+$ ]] && [ "${VehicleNum}" -gt 0 ]; then
            if [ "${VehicleNum}" -gt "${MAX_VEHICLE}" ]; then
                echo "The vehicle number is too large, which may cause a crash"
                read -r -p "Press Enter to continue..."
            else
                break
            fi
        else
            echo "Not a positive integer"
        fi
    done
fi

if ! [[ "${VehicleNum}" =~ ^[0-9]+$ ]] || [ "${VehicleNum}" -le 0 ]; then
    echo "[ERROR] RFV_VEHICLE_NUM must be a positive integer: ${VehicleNum}" >&2
    exit 1
fi

VehicleTotalNum=$((VehicleNum + START_INDEX - 1))
if [ -z "${TOTOAL_COPTER+x}" ]; then
    TOTOAL_COPTER="${VehicleTotalNum}"
fi

sqrtNum=1
squareNum=1
while [ "${squareNum}" -le "${TOTOAL_COPTER}" ]; do
    squareNum=$((sqrtNum * sqrtNum))
    if [ "${squareNum}" -gt "${TOTOAL_COPTER}" ]; then
        break
    fi
    sqrtNum=$((sqrtNum + 1))
done

pkill -x CopterSim || true
pkill -f "CopterSim" || true
echo "Kill all CopterSims"

if [ ! -d "${PSP_PATH}/CopterSim" ]; then
    echo "[ERROR] CopterSim directory not found: ${PSP_PATH}/CopterSim" >&2
    exit 1
fi

cd "${PSP_PATH}/CopterSim"

cntr="${START_INDEX}"
endNum=$((VehicleTotalNum + 1))

while [ "${cntr}" -lt "${endNum}" ]; do
    PosXX=$(( ((cntr - 1) / sqrtNum) * VEHICLE_INTERVAL + ORIGIN_POS_X ))
    PosYY=$(( ((cntr - 1) % sqrtNum) * VEHICLE_INTERVAL + ORIGIN_POS_Y ))
    ./CopterSim 1 "${cntr}" "${CLASS_3D_ID}" "${DLLModel}" "${SimMode}" "${UE4_MAP}" "${IS_BROADCAST}" "${PosXX}" "${PosYY}" "${ORIGIN_YAW}" 1 "${UDPSIMMODE}" &
    sleep 1
    cntr=$((cntr + 1))
done

# If IS_BROADCAST is a non-zero numeric value, PX4 scripts expect it as 1.
if [[ "${IS_BROADCAST}" =~ ^[0-9]+$ ]] && [ "${IS_BROADCAST}" -gt 0 ]; then
    IS_BROADCAST=1
fi

echo "Starting PX4 Build"
cd "${PSP_PATH_LINUX}/Firmware"
if [ -f "./BkFile/EnvOri.sh" ]; then
    # shellcheck disable=SC1091
    source "./BkFile/EnvOri.sh"
fi

make px4_sitl_default
echo "${VehicleNum} ${START_INDEX} ${PX4SitlFrame}"
./Tools/sitl_multiple_run_rfly.sh "${VehicleNum}" "${START_INDEX}" "${PX4SitlFrame}"

read -r -p "Press any key to exit" aa

pkill -x px4 || true
pkill -x CopterSim || true
pkill -x QGroundControl || true
pkill -x RflySim3D || true

echo "Start End."
