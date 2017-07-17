#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include <asm/types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#ifdef  CONFIG_X86
#include <asm/mtrr.h>
#endif
#ifdef CONFIG_ARM
#include <mach/hardware.h>
#endif

static atomic_t open_count = ATOMIC_INIT(0);

static int goldfish_sensor_probe(struct platform_device *pdev);
static int goldfish_sensor_remove(struct platform_device *pdev);

static struct platform_driver goldfish_sensor_driver = { 
        .probe = goldfish_sensor_probe,
        .remove = goldfish_sensor_remove,
        .driver = { 
        .name = "goldfish_sensor"
        }   
};

struct goldfish_sensor_data {
	void __iomem *reg_base;
	int irq;
	wait_queue_head_t wait;
	int32_t __iomem read_buffer[9];
	uint32_t status;
	spinlock_t lock;

};

#define GOLDFISH_SENSOR_READ(data, addr)   (readl(data->reg_base + addr))
#define GOLDFISH_SENSOR_WRITE(data, addr, x)   (writel(x, data->reg_base + addr))
#define READ_BUFFER_SIZE        36

static struct goldfish_sensor_data *sensor_data;

enum {
	/* status register */
	INT_ENABLE = 0x00,

	ACCEL_X = 0x04,
	ACCEL_Y = 0x08,
	ACCEL_Z = 0x0C,

	COMPASS_X = 0x010,
	COMPASS_Y = 0x014,
	COMPASS_Z = 0x018,

	GYRO_X = 0x1C,
	GYRO_Y = 0x20,
	GYRO_Z = 0x24,

	ACCEY_STATUS_CHANGED = 1U << 0,
	COMPASS_STATUS_CHANGED = 1U << 1,
	GYRO_STATUS_CHANGED = 1U << 2,
	SENSOR_INT_MASK = ACCEY_STATUS_CHANGED | COMPASS_STATUS_CHANGED | GYRO_STATUS_CHANGED,
};

static ssize_t goldfish_sensor_read(struct file *fp, char __user *buf,
                                                        size_t count, loff_t *pos)
{
        struct goldfish_sensor_data* data = fp->private_data;
        int length;
        int result = 0;
	
	printk(KERN_ERR "goldfish_sensor_read enter\n");
        while ( count > 0) {
                length = (count > READ_BUFFER_SIZE ? READ_BUFFER_SIZE : count);
                wait_event_interruptible(data->wait, (data-> status & SENSOR_INT_MASK));
		data->read_buffer[0] = GOLDFISH_SENSOR_READ(data, ACCEL_X);
		data->read_buffer[1] = GOLDFISH_SENSOR_READ(data, ACCEL_Y);
		data->read_buffer[2] = GOLDFISH_SENSOR_READ(data, ACCEL_Z);
		data->read_buffer[3] = GOLDFISH_SENSOR_READ(data, COMPASS_X);
                data->read_buffer[4] = GOLDFISH_SENSOR_READ(data, COMPASS_Y);
                data->read_buffer[5] = GOLDFISH_SENSOR_READ(data, COMPASS_Z);
		data->read_buffer[6] = GOLDFISH_SENSOR_READ(data, GYRO_X);
                data->read_buffer[7] = GOLDFISH_SENSOR_READ(data, GYRO_Y);
                data->read_buffer[8] = GOLDFISH_SENSOR_READ(data, GYRO_Z);

                /* copy data to user space */
                if (copy_to_user(buf, data->read_buffer, length))
                {
                        printk("copy_to_user failed!\n");
                        return -EFAULT;
                }

                result += length;
                buf += length;
                count -= length;
        }

        return result;
}



static ssize_t goldfish_sensor_write(struct file *fp, const char __user *buf,
                                                         size_t count, loff_t *pos)
{
	return 0;
}


static int goldfish_sensor_open(struct inode *ip, struct file *fp)
{
        if (!sensor_data)
                return -ENODEV;

        if (atomic_inc_return(&open_count) == 1)
        {
                fp->private_data = sensor_data;
                return 0;
        }
        else
        {
                atomic_dec(&open_count);
                return -EBUSY;
        }
}

static int goldfish_sensor_release(struct inode *ip, struct file* fp)
{
        atomic_dec(&open_count);
        return 0;
}



static long goldfish_sensor_ioctl(struct file* fp, unsigned int cmd, unsigned long arg)
{
	return 0;
}


static irqreturn_t goldfish_sensor_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct goldfish_sensor_data *data = dev_id;
	uint32_t status;

	printk(KERN_ERR "goldfish sensor irq enter\n");
	spin_lock_irqsave(&data->lock, irq_flags);
	/* read status flags, which will clear the interrupt */
	status = GOLDFISH_SENSOR_READ(data, INT_ENABLE);
	uint32_t sensorChanged = status & SENSOR_INT_MASK;
	if(sensorChanged != 0)
	{
		printk(KERN_ERR "goldfish sensor irq wakeup wait, status: %x\n", status);
		data->status = status;
		wake_up(&data->wait);
	}

	spin_unlock_irqrestore(&data->lock, irq_flags);
	return status ? IRQ_HANDLED : IRQ_NONE;
}

/* file operations for /dev/goldfish_sensor */
static struct file_operations goldfish_sensor_fops = {
        .owner = THIS_MODULE,
        .read = goldfish_sensor_read,
        .write = goldfish_sensor_write,
        .open = goldfish_sensor_open,
        .release = goldfish_sensor_release,
        .unlocked_ioctl = goldfish_sensor_ioctl,

};

static struct miscdevice goldfish_sensor_device = { 
        .minor = MISC_DYNAMIC_MINOR,
        .name = "goldfish_sensor",
        .fops = &goldfish_sensor_fops,
};


static int goldfish_sensor_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct goldfish_sensor_data *data;

	printk(KERN_ERR "goldfish sensor prob enter \n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);
	init_waitqueue_head(&data->wait);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		printk(KERN_ERR "%s: platform_get_resource failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_io_base;
	}
#if defined(CONFIG_ARM)
	data->reg_base = (void __iomem *)IO_ADDRESS(r->start - IO_START);
#elif defined(CONFIG_X86) || defined(CONFIG_MIPS)
	data->reg_base = ioremap(r->start, r->end - r->start + 1);
#else
#error NOT SUPPORTED
#endif

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		printk(KERN_ERR "%s: platform_get_irq failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_irq;
	}

	ret = request_irq(data->irq, goldfish_sensor_interrupt, IRQF_SHARED, pdev->name, data);
	if (ret)
		goto err_request_irq_failed;
	if((ret = misc_register(&goldfish_sensor_device)))
        {
                printk("misc_register returned %d in goldfish_sensor_init\n", ret);
                goto err_misc_register_failed;
        }

	platform_set_drvdata(pdev, data);
	sensor_data = data;
	GOLDFISH_SENSOR_WRITE(data, INT_ENABLE, SENSOR_INT_MASK);
	printk(KERN_ERR "goldfish sensor probe exit \n");
	return 0;

err_misc_register_failed:
err_request_irq_failed:
err_no_irq:
#if defined(CONFIG_ARM)
#elif defined(CONFIG_X86) || defined(CONFIG_MIPS)
	iounmap(data->reg_base);
#else
#error NOT SUPPORTED
#endif
err_no_io_base:
	kfree(data);
err_data_alloc_failed:
	return ret;

}

static int goldfish_sensor_remove(struct platform_device *pdev)
{
	misc_deregister(&goldfish_sensor_device);
	struct goldfish_sensor_data *data = platform_get_drvdata(pdev);

	free_irq(data->irq, data);
	kfree(data);
	sensor_data = NULL;
	return 0;
}


static int __init goldfish_sensor_init(void)
{
	printk(KERN_ERR "goldfish sensor init \n");
	return platform_driver_register(&goldfish_sensor_driver);
}

static void __exit goldfish_sensor_exit(void)
{
	printk(KERN_ERR "goldfish sensor exit\n");
	platform_driver_unregister(&goldfish_sensor_driver);
}

module_init(goldfish_sensor_init);
module_exit(goldfish_sensor_exit);

MODULE_AUTHOR("Yonghui Rao raoyonghui0630@gmail.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Sensor driver for the Goldfish emulator");


