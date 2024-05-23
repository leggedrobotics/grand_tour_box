#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sizes.h>
#include <linux/gpio.h> 
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ktime.h>
//#include <linux/time.h>

// Constants
/*****************************************/
#define DRV_NAME                "time_stamper"
#define N_BUFFER_ENTRIES        1024
#define READ_CHARACTER_LIMIT    1000
#define IRQ_GPIO                325      // P9_23
#define IRQ_EDGE                IRQF_TRIGGER_RISING     // specify the edge of the irq

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Mueller");
MODULE_DESCRIPTION("Raspberry Pi kernel module to timestamp GPIO events via IRQ");
MODULE_VERSION("0.02");

// Typedefs
/*****************************************/
struct TimestampRingBuffer {
    int head;
    int tail;
    struct timespec64 bufferentries[N_BUFFER_ENTRIES];
};

// function prototypes
/*****************************************/
static ssize_t ts_buffer_readout(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t ts_buffer_clear(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static irqreturn_t ts_timestamp_handler(int irq, void* irq_data);

// Global variables
/*****************************************/
static struct kobject *ts_kobject;  
static struct TimestampRingBuffer ringbuffer;
static unsigned int irqGPIO = IRQ_GPIO;
static unsigned int irqNumber;
static unsigned int toggle = 0;
// attributes for the sysfs communication, which functions are called by read or write access of the sysfs file
static struct kobj_attribute ts_buffer_attribute =__ATTR(ts_buffer, 0664, ts_buffer_readout, ts_buffer_clear);

// Main
/*****************************************/

/**
 * is called by an GPIO interrupt, und just saves the current time into the buffer
*/
static irqreturn_t ts_timestamp_handler(int irq, void* irq_data)
{
    /// save the current time in the ringbuffer
    ktime_get_real_ts64( &(ringbuffer.bufferentries[ringbuffer.head]) );

    /// increment the head counter
    ringbuffer.head = (ringbuffer.head+1) % N_BUFFER_ENTRIES;

    /// when head overtakes tail, increment tail as well
    if(ringbuffer.head == ringbuffer.tail)
        ringbuffer.tail = (ringbuffer.tail+1) % N_BUFFER_ENTRIES;

    return IRQ_HANDLED;
}

/**
 * Returns the entries of the buffer (event timestamps) in string format
*/
static ssize_t ts_buffer_readout(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t r = 0; 

    /// fill the requested buffer with the timestamps in the ringbuffer until all is read or a certain limit is reached
    while(r < READ_CHARACTER_LIMIT && ringbuffer.tail != ringbuffer.head){
        r += sprintf(buf + r, "%lld.%09ld\n", ringbuffer.bufferentries[ringbuffer.tail].tv_sec, ringbuffer.bufferentries[ringbuffer.tail].tv_nsec);
        ringbuffer.tail = (ringbuffer.tail+1) % N_BUFFER_ENTRIES;
    }

    return r;
}

/**
 * Clears the buffer
*/
static ssize_t ts_buffer_clear(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    ringbuffer.head = 0;
    ringbuffer.tail = ringbuffer.head;

    return count;
}

/**
 * Init function of the LKM, is called at startup
*/
static int __init ts_init(void) 
{
    int err;

    /// create sysfs kernel object folder for the file to access in the file system under /sys/kernel/time_stamper/
    ts_kobject = kobject_create_and_add(DRV_NAME, kernel_kobj);
    if(!ts_kobject)
        return -ENOMEM;

    /// create the actual file for access the buffer functions (/sys/kernel/time_stamper/ts_buffer)
    err = sysfs_create_file(ts_kobject, &ts_buffer_attribute.attr);
    if (err) {
        printk(KERN_ALERT DRV_NAME " : failed to create the sysfs file for the buffer access\n");
        return err;
    }       
    /// setup irq gpio and set it to input
    err = gpio_request_one(irqGPIO, GPIOF_IN, DRV_NAME " irq");
    if (err < 0) {
        printk(KERN_ALERT DRV_NAME " : failed to request IRQ pin %d.\n", irqGPIO);
        return -1;
    }

    /// find out, which IRQ number belongs to the input pin
    irqNumber = gpio_to_irq(irqGPIO);
    if (irqNumber < 0) {
        printk(KERN_ALERT DRV_NAME " : failed to get IRQ for pin %d.\n", irqGPIO);
        gpio_free(irqGPIO);
        return -1;
    }

    /// enable the interrupt as a specified edge, connect it to the handler function and send the irqNumber as cookie
    err = request_any_context_irq(irqNumber, ts_timestamp_handler, IRQ_EDGE, DRV_NAME, (void*)irqNumber);
    if (err < 0) {
        printk(KERN_ALERT DRV_NAME " : failed to enable IRQ %d for pin %d.\n",irqNumber, irqGPIO);
        gpio_free(irqGPIO);
        return -1;
    }

    /// init the timestamp ring-buffer
    ringbuffer.head = 0;
    ringbuffer.tail = ringbuffer.head;

    printk(KERN_INFO DRV_NAME " : GPIO IRQ Timestamper initialized and started\n");
    return 0;
}

/**
 * Exit function of the LKM, is called when the LKM ends
*/
static void __exit ts_exit(void) 
{
    /// remove the kobject and all its sub-files
    kobject_put(ts_kobject);

    /// remove all IRQs
    free_irq(irqNumber, (void*)irqNumber);

    /// remove GPIOs
    gpio_free(irqGPIO);
    printk(KERN_INFO DRV_NAME " : GPIO IRQ Timestamper ended\n");
}

module_init(ts_init);
module_exit(ts_exit);
