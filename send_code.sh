# --exclude='*.sh' --exclude='*.bash'
# --exclude='install/'
# --exclude='build/'
# --exclude='log/'

sshpass -p 'xline' rsync -av -P -z --exclude='.vscode/' --exclude='log/' --exclude='build/' --exclude='install/' --exclude='other/' --exclude='.git/' --exclude='.gitignore'  /root/xline_ws/xline_base_controller xline@192.168.43.85:/home/xline/xline_ws

