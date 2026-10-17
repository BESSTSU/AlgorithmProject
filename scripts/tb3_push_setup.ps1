# Copy tb3_sbc_setup.sh to a TurtleBot3 Raspberry Pi and run it inside tmux.
# The user types the robot password (once, for the key copy) and the sudo password.
# Usage: .\scripts\tb3_push_setup.ps1 user@ip [LDS-01|LDS-02]
param([Parameter(Mandatory)][string]$Target, [string]$Lds = "")

# 1) authorize this PC's SSH key on the robot so later checks need no password
Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh $Target "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && sort -u -o ~/.ssh/authorized_keys ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
if (-not $?) { exit 1 }

# 2) copy the setup script
scp "$PSScriptRoot\tb3_sbc_setup.sh" "${Target}:~/tb3_sbc_setup.sh"
if (-not $?) { exit 1 }

# 3) run it in tmux so the build survives an SSH disconnect (reattach: ssh -t <target> tmux attach -t sbc)
ssh -t $Target "tmux new-session -A -s sbc 'bash ~/tb3_sbc_setup.sh $Lds 2>&1 | tee -a ~/sbc_setup.log; exec bash'"
