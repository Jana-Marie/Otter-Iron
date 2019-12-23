# Otter-Iron

Replacement PCB for TS100, adds USB-C PD to every hackers favorite tool!

Hardware is finished and working, software V0.1!

## Flashing

To flash the Otter-Iron short the DFU-jumper shown in the image below while plugging it into your computer. This will put the iron into DFU mode allowing you to flash it via DFU/USB. Run this command to flash the downloaded binary.

    $ dfu-util -a 0 -s 0x08000000:leave -D build/firmware.bin

![dfu](images/dfu.png)

## Video

[Video (clickme)](https://twitter.com/JanHenrikH/status/1208867279540232192)

## Images


![3](images/3.jpg)
![4](images/4.jpg)

![Front](images/front.png)
![Back](images/back.png)

## Todo software

 - [ ] Write a better regulation
 - [ ] Current measurement
 - [ ] Store last setpoint
 - [ ] Better Fonts/UI
 - [ ] PD-Profile cycling
 
## Todo Hardware

Todo next hardware revision:
 - [ ] Add barrel jack
