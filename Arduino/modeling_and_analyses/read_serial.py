import serial 
import time 
import pandas

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0', baudrate = 115200, timeout=1)
    ser.reset_input_buffer()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

echo ""
echo "******* Add User to dialout,tty, uucp, plugdev groups *******"
echo ""

sudo usermod -a -G tty $1
sudo usermod -a -G dialout $1
sudo usermod -a -G uucp $1
sudo groupadd plugdev
sudo usermod -a -G plugdev $1


acmrules () {

    echo ""
    echo "# Setting serial port rules"
    echo ""

    cat <<EOF
    "KERNEL="ttyUSB[0-9]*", TAG+="udev-acl", TAG+="uaccess", OWNER="$1"
   "KERNEL="ttyACM[0-9]*", TAG+="udev-acl", TAG+="uaccess", OWNER="$1"
   "KERNEL="ttyS[0-9]*", TAG+="udev-acl", TAG+="uaccess", OWNER="$1"
   EOF

}