sudo modprobe can  &&
sudo modprobe can_raw &&
sudo modprobe mttcan  &&
sudo ip link set can1 type can bitrate 500000 sjw 4 berr-reporting on loopback off &&
sudo ip link set up can1 