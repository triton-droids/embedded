# Sourced by run_one.sh and preflight.sh: where the WSL and Jetson commands run.
# On the WSL host itself (it owns 192.168.50.1) WSL commands run locally and the
# Jetson is reached over the direct link; elsewhere (the laptop) both go over SSH.
# accept-new: the first login over the link records the Jetson host key without a prompt.
SSH="ssh -o ConnectTimeout=10 -o ServerAliveInterval=15 -o BatchMode=yes -o StrictHostKeyChecking=accept-new"
if ip -4 -br addr 2>/dev/null | grep -q '192\.168\.50\.1/'; then
  WSL=${WSL:-local}; JET=${JET:-td@192.168.50.3}
else
  WSL=${WSL:-rcli@192.168.1.129}; JET=${JET:-td@100.71.186.19}
fi
# pipefail: a check such as "ping ... | tail -1" must fail when ping fails.
on_wsl() { if [ "$WSL" = local ]; then bash -o pipefail -c "$1"; else $SSH $WSL "set -o pipefail; $1"; fi; }
on_jet() { $SSH $JET "set -o pipefail; $1"; }
