sudo slcan_attach -f -s6 -o /dev/arduino_can
sudo slcand -S 1000000 arduino_can can0  
sudo ip link set can0 up
