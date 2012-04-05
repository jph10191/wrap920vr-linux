# wrap920vr-linux

wrap920vr-linux is a Linux driver for the Vuzix Wrap 920 Tracker 6TC. It uses the hidraw0 device which is created by the linux kernel when connecting the VR-glasses to the USB port. This userland driver parses the binary data from the hidraw0 device and calculates the according angles pitch, roll and yaw. The Vuzix Wrap 920 Tracker 6TC consists of three sensors: a gyroscope, a magnetometer and an accelerometer. Pitch, roll and yaw are calculated using different sensors. To prevent errors an error correcting approach is implemented by using two sensors for pitch and yaw.   

## Requirements

## Example application

## Contribution


