# --exclude='*.sh' --exclude='*.bash'

sshpass -p 'xline' rsync -av -P -z --exclude='.vscode/' --exclude='other/' --exclude='.git/' --exclude='.gitignore'  /root/xline_ws/xline_base_controller xline@192.168.43.85:/home/xline/xline_ws

