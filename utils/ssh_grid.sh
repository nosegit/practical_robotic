#!/bin/bash

# --- Configuration ---
TARGET_IP="${PI_IP:-192.168.1.100}"
USER="pi"
PASS="raspberry"
WORK_DIR="WS/Rpi"
SESSION="robot_mission"

# Panel Commands
CMD_0="python Driver/robot_service.py"
CMD_1="cd Vision && python camera_server_vid.py"
CMD_2="echo 'Panel 2 Ready'"
CMD_3="python Vision/camera_viewer.py"

# --- Step 1: Create the Robust Helper Script ---
# KEY FIX: We added 'sleep 1' and stricter prompt matching (pi@)
cat << 'EOF' > /tmp/login_helper.exp
#!/usr/bin/expect

set ip [lindex $argv 0]
set user [lindex $argv 1]
set pass [lindex $argv 2]
set work_dir [lindex $argv 3]
set cmd [lindex $argv 4]
set timeout 20

# Start SSH
spawn ssh -XC -o StrictHostKeyChecking=no $user@$ip

# Handle Password & Initial Login
expect {
    # If asked for fingerprint
    "yes/no" { send "yes\r"; exp_continue }
    
    # If asked for password
    "*assword:*" { send "$pass\r"; exp_continue }
    
    # SUCCESS CONDITION: Look for the specific username "pi@" in the prompt
    # This prevents triggering early on random '$' signs in the login banner
    -re "pi@.*" { }
    
    timeout { puts "Connection Timed Out"; exit 1 }
}

# FORCE WAIT: Let the prompt settle and banners finish
sleep 1

# Send Change Directory
send "cd $work_dir\r"

# Wait for the prompt again (pi@...)
expect -re "pi@.*"

# Send the Python Command
send "$cmd\r"

# Hand control to user
interact
EOF
chmod +x /tmp/login_helper.exp

# --- Step 2: Setup Tmux Layout ---

tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# Start Session (Window 0, Pane 0)
tmux new-session -d -s $SESSION

# Create 2x2 Grid (0=TL, 1=BL, 2=TR, 3=BR)
# 1. Split Left/Right
tmux split-window -h -t $SESSION:0.0
# 2. Split Left Side (Top/Bottom)
tmux split-window -v -t $SESSION:0.0
# 3. Split Right Side (Top/Bottom)
tmux split-window -v -t $SESSION:0.2

# --- Step 3: Launch with Delays ---

echo "Launching Panel 1 (Top Left)..."
tmux send-keys -t $SESSION:0.0 "/tmp/login_helper.exp $TARGET_IP $USER $PASS \"$WORK_DIR\" \"$CMD_0\"" C-m

echo "Launching Panel 2 (Bottom Left)..."
tmux send-keys -t $SESSION:0.1 "/tmp/login_helper.exp $TARGET_IP $USER $PASS \"$WORK_DIR\" \"$CMD_2\"" C-m

echo "Launching Panel 3 (Top Right)..."
tmux send-keys -t $SESSION:0.2 "/tmp/login_helper.exp $TARGET_IP $USER $PASS \"$WORK_DIR\" \"$CMD_1\"" C-m

echo "Launching Panel 4 (Bottom Right)..."
tmux send-keys -t $SESSION:0.3 "/tmp/login_helper.exp $TARGET_IP $USER $PASS \"$WORK_DIR\" \"$CMD_3\"" C-m

# --- Attach ---
tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION
