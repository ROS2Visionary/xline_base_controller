# --exclude='*.sh' --exclude='*.bash'
# --exclude='install/'
# --exclude='build/'
# --exclude='log/'

sshpass -p 'xline' rsync -av -P -z --exclude='.vscode/' --exclude='.claude/' --exclude='log/' --exclude='build/' --exclude='install/' --exclude='other/' --exclude='.git/' --exclude='.gitignore'  /root/xline_ws/xline_base_controller xline@192.168.1.123:/home/xline/zyq_ws

