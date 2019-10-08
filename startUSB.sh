sudo chmod 777 /dev/ttyUSB0
sudo su -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
