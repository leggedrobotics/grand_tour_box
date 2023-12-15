# time_stamper kernel module
Timestamp kernel module for the Raspberry Pi (and possible other devices)

## introduction
To be able to get the exact knowledge when an event happened is most of the time crucial, especially in cases of sensor fusion or process supervision. Most of the control systems use an embedded system where there are no hardware timers to use for this and software timers are way to unreliable for sub-millisecond resolution. Mostly in the robotics environment, the time stamping is done in an external microcontroller with dedicated hardware timers. While this works well, it is mostly cumbersome to have an additional dedicated hardware just for that and the problem remains to synchronize the embedded system time to the one of the microcontroller (if required). 

The Raspberry Pi does have GPIO pins which could generate a direct hardware interrupt in a kernel module. This could be used to generate timestamps, and this kernel module is written to just do that. The interrupt could be specified during compile time to be stamped on specified pins. The time stamped times could then be read out via file from an internal ring-buffer.

## time_stamper.c setup
 - N_BUFFER_ENTRIES: specify the number of ring-buffer entries for time stamps storage
 - N_BUFFER_ENTRIES: the limit of how many characters are printed when the ring-buffer is read out; it is possible that the buffer needs to be read out a couple of times if it is full if this value is small
 - IRQ_GPIO: GPIO number of the pin which needs to be time-stamped
 - TRIGGER_GPIO: GPIO which toggles every time a timestamp was happening; could be used to validate the system or to just give a visual cue e.g. on a LED (enabled by TOGGLE_ENABLE)
 - IRQ_EDGE: specifies the edge on the IRQ_GPIO, where the interrupt should be happening; possibilities are IRQF_TRIGGER_RISING and IRQF_TRIGGER_FALLING
 - TOGGLE_ENABLE: by specifying this define, the TRIGGER_GPIO function will be enabled; disable it when only the time stamping should be used without pin toggling when the event happened

## compile
first, you need the correct packages to build the kernel module. Install them by
`sudo apt install make gcc`

Additionally, the kernel headers are required. For Ubuntu, install them by 
`apt-get install build-essential linux-headers-\`uname -r\``

## usage
Start the kernel module with
`sudo insmod time_stamper.ko`

To check if it is correctly loaded, check the kernel log:
`sudo dmesg`

And if the module is still loaded 
`lsmod | grep "time_stamper"`

To remove it, use the following line:
`sudo rmmod time_stamper`

The kernel module creates a file "ts_buffer" in "/sys/kernel/time_stamper/". There, the timestamped times could be read out, e.g. 
`cat /sys/kernel/time_stamper/ts_buffer`
To clear the buffer, just anything could be written to it to clear the buffer
`echo "1" | /sys/kernel/time_stamper/ts_buffer`


## verification
