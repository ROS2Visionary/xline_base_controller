# --exclude='*.sh' --exclude='*.bash'
# --exclude='install/'
# --exclude='build/'
# --exclude='log/'

# sshpass -p 'xline' rsync -av -P -z --exclude='.vscode/' --exclude='.claude/' --exclude='log/' --exclude='build/' --exclude='install/' --exclude='other/' --exclude='.git/' --exclude='.gitignore'  \
#  /root/xline_ws/xline_base_controller/src/xline_base_controller \
#  /root/xline_ws/xline_base_controller/src/xline_cmd_vel_mux \
#  /root/xline_ws/xline_base_controller/src/xline_follow_controller \
#  /root/xline_ws/xline_base_controller/src/xline_inkjet_printer \
#  /root/xline_ws/xline_base_controller/src/xline_msgs \
#  /root/xline_ws/xline_base_controller/src/xline_obstacle_detector \
#  /root/xline_ws/xline_base_controller/src/xline_localization \
#  xline@192.168.0.123:/home/xline/zyq_ws

sshpass -p 'xlinerobot' rsync -av -P -z \
  -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o PreferredAuthentications=password -o PubkeyAuthentication=no" \
  --exclude='.vscode/' \
  --exclude='kill.sh' \
  --exclude='run_xline.sh/' \
  --exclude='.claude/' \
  --exclude='log/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='other/' \
  --exclude='.git/' \
  --exclude='.gitignore' \
  /root/xline_ws/xline_base_controller \
  cat@192.168.31.228:/home/cat/xline_ws
