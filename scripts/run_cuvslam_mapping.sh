#!/usr/bin/env bash
# Runs the D435i (stereo IR, emitter ON) + cuVSLAM pipeline and RViz for live mapping.
# Pure visual odometry (no IMU fusion) - the D435i's Motion Module currently locks up
# the RSUSB backend inside this container, see session notes. Re-run any time to start
# a fresh container session; call reset_map.sh afterwards to clear an existing map.

set -euo pipefail

CONTAINER=isaac_ros_dev-x86_64-container
RVIZ_CFG_SRC=/opt/ros/humble/share/isaac_ros_visual_slam/rviz/realsense.cfg.rviz
RVIZ_CFG=/tmp/realsense_live.rviz
EMITTER_CFG_SRC=/opt/ros/humble/share/isaac_ros_realsense/config/realsense_stereo.yaml
EMITTER_CFG=/tmp/realsense_stereo_emitter.yaml

echo "==> Making sure the container is running"
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    docker start "$CONTAINER"
    sleep 2
fi

echo "==> Refreshing X11 auth (needed after host reboot/relogin)"
xhost +SI:localuser:"$USER" >/dev/null 2>&1 || true
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    cp "$XAUTHORITY" "$HOME/.Xauthority"
fi

echo "==> Killing any leftover ROS/RealSense processes in the container"
# NOTE: patterns use bracket-obfuscation (e.g. 'ros[2] launch') so pkill -f doesn't
# match its own invocation's command-line text and kill its own parent shell.
docker exec "$CONTAINER" bash -c "
  pkill -9 -f 'ros[2] launch' 2>/dev/null
  pkill -9 -f 'component_containe[r]' 2>/dev/null
  pkill -9 -f 'realsense2_camera_nod[e]' 2>/dev/null
  pkill -9 -f 'rviz[2]' 2>/dev/null
  sleep 1
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
  true
"

echo "==> Restarting ros2 daemon (stale daemon after clearing shared memory breaks 'ros2 topic/node' CLI calls)"
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  ros2 daemon stop 2>/dev/null
  ros2 daemon start
"

echo "==> Writing RealSense config with IR emitter enabled"
docker exec "$CONTAINER" bash -c "
  cp $EMITTER_CFG_SRC $EMITTER_CFG
  sed -i 's/emitter_enabled: 0/emitter_enabled: 1/' $EMITTER_CFG
"

echo "==> Launching RealSense stereo + cuVSLAM (pure VO, base_frame=camera_link)"
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
    launch_fragments:=realsense_stereo_rect,visual_slam \
    base_frame:=camera_link \
    'camera_optical_frames:=[camera_infra1_optical_frame, camera_infra2_optical_frame]' \
    realsense_config_file:=$EMITTER_CFG \
    > /tmp/vslam_run.log 2>&1
"

echo "==> Waiting for cuVSLAM tracker to initialize..."
for i in $(seq 1 20); do
    sleep 1
    if docker exec "$CONTAINER" grep -q "cuVSLAM tracker was successfully initialized" /tmp/vslam_run.log 2>/dev/null; then
        echo "    tracker initialized after ${i}s"
        break
    fi
done

echo "==> Preparing RViz config pointed at the live topics"
docker exec "$CONTAINER" bash -c "
  cp $RVIZ_CFG_SRC $RVIZ_CFG
  sed -i 's#/camera/infra1/image_rect_raw#/infra1/image_rect_raw_mono#; s#/camera/infra2/image_rect_raw#/infra2/image_rect_raw_mono#' $RVIZ_CFG
  sed -i '/Normalize Range: true/s/true/false/' $RVIZ_CFG
"

echo "==> Launching RViz"
docker exec -d "$CONTAINER" bash -c "
  export XAUTHORITY=/home/admin/.Xauthority
  source /opt/ros/humble/setup.bash
  rviz2 -d $RVIZ_CFG > /tmp/rviz_run.log 2>&1
"

sleep 3
echo "==> Tracking status:"
docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /visual_slam/status --field vo_state --once" 2>&1 || echo "    (no status yet - give it a few more seconds)"

echo ""
echo "Done. Move the camera to build the map; check RViz for the trajectory/point cloud."
echo "To clear the map and start over: bash reset_map.sh"
