# Send Telemetry via Ethernet in Raspberry Pi or Jetson

In this tutorial, we will use an UART connection between a onboard computer, Raspberry Pi or Jetson,
and Pixhawk to send telemetry data to a ground station (QGroundControl) via Mavlink-Router.

The UART connection could be Pixhawk connected directly to the USB hub in the Computer or connected through a serial connection in pins (soldered cable). See [correct pin configuration](https://github.com/ViniciusAbrao/px4_ros2_xrcedds/blob/master/tutorial/tutorial.md#mavlink-communication)

## Pixhawk

Connect Pixhawk to QGroundControl via USB cable.

In Parameters menu, set these:

![mavlink menu uart](assets/mavlink_menu_uart.png)
![uart baudrate](assets/uart_baudrate.png)

Reboot Pixhawk and disconnect the USB cable.

## Computer (Jetson Orin Nano or Raspberry pi)

Pre-requisites:
- [Ubuntu 22.04 install](../raspberry_pi/OS-install.md)

- [Mavlink-Router install](https://github.com/mavlink-router/mavlink-router)

### TLDR install for Mavlink-Router

Clone and install the directory:

```shell
#required packages:
#sudo apt install git meson ninja-build pkg-config gcc g++ systemd

git clone https://github.com/mavlink-router/mavlink-router
cd mavlink-router
meson setup build
ninja -C build
sudo ninja -C build install
```

Configure Mavlink-Router to run on port 14540 (standard for Onboard-Computer):

```shell
#this config file name might change slightly depending whether it is on rasp or jetson

sudo nano /etc/mavlink-router/pix-uart.conf
```

Paste and then save the file:

```conf
[UartEndpoint alpha]
  #ACM0 for usb, ttyAMA0 and serial0 for serial
  Device = /dev/ttyACM0
  Baud = 57600

[UdpEndpoint ground]
  Mode=Server
  #Here, it needs the receiver address, usually qGround-Control's ip
  Address=0.0.0.0
  Port=14540

```

Now run Mavlink-Router, which should output Server and Client creation:

```shell
#once again, adapt to your config file name
mavlink-routerd -c /etc/mavlink-router/pix-uart.conf
```

## QGroundControl

Go to "Application Settings" > "Comm links" > "Add" > and configure this:

![qground_comm_uart](assets/qground_comm_links-uart.png)

Click "OK" > "Connect".

If the connection was proper, QGroundControl should now be connected to the Pixhawk.

## References

[Correct Serial Pins Configuration](https://github.com/ViniciusAbrao/px4_ros2_xrcedds/blob/master/tutorial/tutorial.md#mavlink-communication)

[Mavlink-Router Documentation and Examples](https://github.com/mavlink-router/mavlink-router)