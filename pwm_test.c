#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/sizes.h>
#include <linux/timer.h>

#define GPIO_PWM_MAJOR 18

// First and last GPIO port numbers for the Raspberry Pi 2
#define FIRST_GPIO_PORT 2
#define LAST_GPIO_PORT 27
#define RPI_GPIO_OUT 18

/* Functions which are called when an user accesses or exits the character device. 
    (Open for the access, release when exiting it) */

ssize_t gpio_pwm_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "Opening gpio_pwm_module...\n");
    return 0;
}

ssize_t gpio_pwm_release(struct inode *inode, struct file *file) {
    printk(KERN_INFO "Releasing gpio_pwm_module...\n");
    return 0;
}

/* Defines a structure enabling the use of a PWM device on the board. */

static struct timer_list timer_period;
static struct timer_list timer_duty;
unsigned long time_duty = (HZ >> 3);
unsigned long time_period = (HZ >> 2);
int value = 1;
int pwm = 0;

/* Handle a PWM device or a GPIO port by using the I/O control:
        - command:   * 0: enable/disable the PWM device
                         * 1: configure the PWM device (duty cycle, period)
                         * between 2 and 27: GPIO port number
                         * 99: set all the GPIO port values to 0
        - arguments: * for the PWM device: duty cycle and period in nanoseconds
                         * for a specific GPIO port: 0 to switch it off, 1 to switch it on, 2 to define it as input, 3 to read its value */

/* The function will exit if any of the parameters is wrong.
    The prototype of the function may vary according of the version of the Linux kernel. */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    int gpio_pwm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#else
    long gpio_pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
    int i;

    switch (cmd) {

        case 30:
            pwm = 1 - pwm;
            printk("pwm %d",pwm);
            gpio_direction_output(RPI_GPIO_OUT, pwm);
            break;

        case 31:
            printk(KERN_INFO "Confguring the PWM device (duty cycle: %d Hz)...\n", (int)arg);
            time_duty  = arg;
            printk("arg %lu",time_duty);
            mod_timer(& timer_duty,jiffies+ time_duty);
            break;
        case 32:
            printk(KERN_INFO "Confguring the PWM device (period: %d Hz)...\n", (int)arg);
            time_period = arg;
            printk("arg %lu",time_period);
            mod_timer(& timer_period,jiffies+ time_period);
            break;

        case 2:
        case 3:
        case 4:
        case 14:
        case 15:
        case 17:
        case 18:
        case 27:
        case 22:
        case 23:
        case 24:
        case 10:
        case 9:
        case 11:
        case 25:
        case 8:
        case 7:
        case 5:
        case 6:
        case 13:
        case 19:
        case 26:
        case 12:
        case 16:
        case 20:
        case 21:
            switch (arg) {

                case 0:
                    if(cmd == RPI_GPIO_OUT)
                        pwm = 0;
                    printk(KERN_INFO "Switching the GPIO port %d off...\n", cmd);
                    gpio_direction_output(cmd, 0);
                    gpio_set_value(cmd, 0);
                    break;

                case 1:
                    if(cmd == RPI_GPIO_OUT)
                        pwm = 0;
                    printk(KERN_INFO "Switching the GPIO port %d on...\n", cmd);
                    gpio_direction_output(cmd, 1);
                    gpio_set_value(cmd, 1);
                    break;

                case 2:
                    if(cmd == RPI_GPIO_OUT)
                        pwm = 0;
                    printk(KERN_INFO "Switching the GPIO port %d as input...", cmd);
                    gpio_direction_input(cmd);
                    break;

                case 3:
                    if(cmd == RPI_GPIO_OUT)
                        pwm = 0;
                    gpio_direction_input(cmd);
                    printk(KERN_INFO "Value of GPIO port %d is %d", cmd, gpio_get_value(cmd));
                    break;

                default:
                    printk(KERN_INFO "GPIO port %d : Unknown argument (%ld)\n", cmd, arg);
                    return -1;
            }
            break;

        case 99:
            printk(KERN_INFO "Setting the value of the GPIO ports to 0...\n");
            for (i = FIRST_GPIO_PORT; i <= LAST_GPIO_PORT; i++){
                if(i == RPI_GPIO_OUT)
                   pwm = 0;
                gpio_direction_output(cmd, 0);
                gpio_set_value(i, 0);
            }
            break;
        case 100:
            printk(KERN_INFO "Setting the value of the GPIO ports to 1...\n");

            for (i = FIRST_GPIO_PORT; i <= LAST_GPIO_PORT; i++){
                if(i == RPI_GPIO_OUT)
                   pwm = 0;
                gpio_direction_output(cmd, 1);
                gpio_set_value(i, 1);
            }
            break;
        default:
            printk(KERN_INFO "Unknown command or GPIO port (%d)\n", cmd);
            return -1;
    }

    return 0;
}

/* Defines the operations done by the module when an user accesses it. */

struct file_operations fops = {
    .open = gpio_pwm_open,
    .release = gpio_pwm_release,

    #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
        .ioctl = gpio_pwm_ioctl
    #else
        .unlocked_ioctl = gpio_pwm_ioctl
    #endif
};

/* The kernel module is written as a character device.
    It needs to be registered when being initialized, and unregistered when being exited. */

static void period_function (unsigned long unused)
{
  if(pwm==1){
     value = 1;
     gpio_set_value(RPI_GPIO_OUT, value);
     mod_timer(& timer_duty,jiffies+ time_duty);
     mod_timer(& timer_period,jiffies+ time_period);
  }
}

static void duty_function (unsigned long unused)
{
  if(pwm==1){
     value = 0;
     gpio_set_value(RPI_GPIO_OUT, value);
  }
}

static int gpio_pwm_init(void) {
    int i;
    printk(KERN_ALERT "Initializing the kernel module for GPIO and PWM ports...\n");
    register_chrdev(GPIO_PWM_MAJOR, "gpio_pwm_module", &fops);

    for (i = FIRST_GPIO_PORT; i <= LAST_GPIO_PORT; i++)
        gpio_direction_output(i,1);

    init_timer(& timer_period);
    timer_period.function = period_function;
    timer_period.data = 0; // non utilise
    timer_period.expires = jiffies+ time_period;
    printk("init timer period");
    add_timer(& timer_period);

    init_timer(& timer_duty);
    timer_duty.function = duty_function;
    timer_duty.data = 0; // non utilise
    timer_duty.expires = jiffies+ time_duty;
    printk("init duty period");
    add_timer(& timer_duty);

    return 0;
}

static int gpio_pwm_exit(void) {
    int i;

    for (i = FIRST_GPIO_PORT; i <= LAST_GPIO_PORT; i++)
        gpio_free(i);

    del_timer(& timer_period);
    del_timer(& timer_duty);
    unregister_chrdev(GPIO_PWM_MAJOR, "gpio_pwm_module");
    printk(KERN_ALERT "Exiting the kernel module for GPIO and PWM ports...\n"); 
    return 0;
}

module_init(gpio_pwm_init);
module_exit(gpio_pwm_exit);
MODULE_LICENSE("GPL");
