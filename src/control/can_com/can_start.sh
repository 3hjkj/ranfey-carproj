sudo modprobe can  &&
sudo modprobe can_raw &&
sudo modprobe mttcan  &&
sudo ip link set can0 type can bitrate 500000 sjw 4 berr-reporting on loopback off &&
sudo ip link set up can0 