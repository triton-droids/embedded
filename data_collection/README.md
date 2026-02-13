sudo modprobe slcan;

sudo slcand -o -c -s6 /dev/ttyACM0 can0;

sudo ip link set can0 down;

sudo ip link set can0 type can bitrate 1000000;

sudo ip link set can0 up;

\# Create a new terminal to verify and monitor CAN traffic
candump -x -t a can0

\# At this point ensure the virtual environment is activated

for i in {1..10}; do cansend can0 $(printf "0600FE%02X#0100000000000000" $i); sleep 0.05; done; echo "Set Mechanical Zeros"

python3 collect_motor_dataset.py \
  --campaign all \
  --root-dir ./motor_dataset \
  --hz 400 \
  --ik-path IK_trajectory \
  --save-csv
