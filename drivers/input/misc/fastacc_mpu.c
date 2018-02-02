/*
 * fastacc_mpu.c - High-speed accelerometer driver for InvenSense MPU devices
 *
 * Copyright (C) 2017 Robert Xiao <brx@cs.cmu.edu>
 *     Future Interfaces Group, Carnegie Mellon University
 *
 * If you use this code, we ask you cite the originating paper, ViBand:
 *
 *   Gierad Laput, Robert Xiao, and Chris Harrison. 2016. ViBand: High-Fidelity
 *   Bio-Acoustic Sensing Using Commodity Smartwatch Accelerometers.
 *   In Proceedings of the 29th Annual Symposium on User Interface Software and
 *   Technology (UIST '16). ACM, New York, NY, USA, 321-333.
 *   DOI: https://doi.org/10.1145/2984511.2984582
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "fastacc_mpu.h"

/* Device driver name - device link is created at /sys/class/misc/$DEVICE_NAME */
#define DEVICE_NAME "fastacc_mpu"

/* Device definitions from inv_mpu_iio.h */
enum inv_devices {
    INV_ITG3500,
    INV_MPU3050,
    INV_MPU6050,
    INV_MPU9150,
    INV_MPU6500,
    INV_MPU9250,
    INV_MPU6XXX,
    INV_MPU9350,
    INV_MPU6515,
};

/* Configuration */
static struct {
    int range;
    int flag;
} mpu_accel_ranges[] = {
    {2, MPU_ACCEL_CONFIG_ACCEL_FS_SEL_2_G},
    {4, MPU_ACCEL_CONFIG_ACCEL_FS_SEL_4_G},
    {8, MPU_ACCEL_CONFIG_ACCEL_FS_SEL_8_G},
    {16, MPU_ACCEL_CONFIG_ACCEL_FS_SEL_16_G},
};
static int mpu_accel_range_default = 3; /* 16 G */

#define POWER_UP_TIME 30 /* msec */
#define MAX_BURST_SIZE 252 /* maximum size of one I2C read */
#define POLL_FIFO_BYTES 168000 /* number of bytes in the big kernel FIFO. Set to a multiple of LCM(2, 4, 6, ..., 16) to accommodate any unit size */
static int mpu_poll_thread_fn(void *data);

/* Device private data structure */
struct mpu_devdata {
    struct i2c_client *client;
    enum inv_devices chiptype;
    struct miscdevice miscdev;

    /* Persistent configuration (saved across power cycles) */
    int accel_range; /* index into mpu_accel_ranges table */
    int fifo_mode; /* FIFO_MODE: 1 = stop on full, 0 = overwrite */

    /* Power state */
    bool powered;
#ifdef CONFIG_OF
    struct regulator *vdd_ana, *vcc_i2c;
#endif

    /* FIFO statistics */
    uint64_t fifo_read_total;
    struct timespec fifo_read_time;

    /* High-speed polling thread */
    struct task_struct *poll_thread;
    struct completion poll_completion;
    int poll_wpos, poll_size; // poll_fifo is valid in the range [poll_wpos-poll_size:poll_wpos], mod POLL_FIFO_BYTES
    u8 poll_fifo[POLL_FIFO_BYTES];
    wait_queue_head_t poll_wq; // poll_wq.lock protects poll_wpos and poll_size too
};
#define MPU_DEV(mpu) (&(mpu)->client->dev)

/* I2C I/O */
static int mpu_read(struct mpu_devdata *mpu, u8 reg, u8 *data, u16 length)
{
    struct i2c_msg msgs[2] = {
        {
            .addr = mpu->client->addr,
            .flags = 0,
            .buf = &reg,
            .len = 1,
        },
        {
            .addr = mpu->client->addr,
            .flags = I2C_M_RD,
            .buf = data,
            .len = length,
        }
    };
    int res;

    if(!mpu->powered) {
        dev_warn(MPU_DEV(mpu), "device is off; refusing read of reg=0x%02x len=%d\n", reg, length);
        return 0;
    }

    res = i2c_transfer(mpu->client->adapter, msgs, 2);

    if (res < 2) {
        if (res >= 0)
            res = -EIO;
        return res;
    }

    return length;
}

static int mpu_read_reg(struct mpu_devdata *mpu, u8 reg)
{
    u8 val;
    int res;

    res = mpu_read(mpu, reg, &val, 1);
    if(res < 0) {
        return res;
    } else {
        return val;
    }
}

static int mpu_read_be16(struct mpu_devdata *mpu, u8 reg)
{
    u16 val;
    int res;

    res = mpu_read(mpu, reg, (u8*)&val, 2);
    if(res < 0) {
        return res;
    } else {
        return be16_to_cpu(val);
    }
}

static int mpu_write_reg(struct mpu_devdata *mpu, u8 reg, u8 data)
{
    u8 tmp[2] = { reg, data };
    struct i2c_msg msg = {
        .addr = mpu->client->addr,
        .flags = 0,
        .buf = tmp,
        .len = 2,
    };
    int res;

    if(!mpu->powered) {
        dev_warn(MPU_DEV(mpu), "device is off; refusing write of reg=0x%02x val=0x%02x\n", reg, data);
        return 0;
    }

    res = i2c_transfer(mpu->client->adapter, &msg, 1);

    if (res < 1) {
        if (res >= 0)
            res = -EIO;
        return res;
    }

    return 0;
}

/* Power control */
static int mpu_poweron(struct mpu_devdata *mpu)
{
    int ret;

#ifdef CONFIG_OF
    ret = regulator_enable(mpu->vdd_ana);
    if(ret) {
        dev_err(MPU_DEV(mpu), "failed to enable vdd_ana\n");
        goto fail_vdd_ana;
    }
    ret = regulator_enable(mpu->vcc_i2c);
    if(ret) {
        dev_err(MPU_DEV(mpu), "failed to enable vcc_i2c\n");
        goto fail_vcc_i2c;
    }

    msleep(POWER_UP_TIME);
#endif

    mpu->powered = true;

    ret = mpu_read_reg(mpu, MPU_WHO_AM_I);
    if (ret < 0) {
        dev_err(MPU_DEV(mpu), "failed to read WHO_AM_I\n");
        goto fail_whoami;
    }

    dev_dbg(MPU_DEV(mpu), "WHO_AM_I = %d (0x%02x)\n", ret, ret);

    mpu_write_reg(mpu, MPU_PWR_MGMT_1, MPU_PWR_MGMT_1_DEVICE_RESET);
    msleep(10);

    mpu_write_reg(mpu, MPU_PWR_MGMT_1, 0
        // | MPU_PWR_MGMT_1_GYRO_STANDBY // disable gyro
        | MPU_PWR_MGMT_1_TEMP_DIS // disable temp sensor
        | MPU_PWR_MGMT_1_CLKSEL_INTERNAL // use internal osc (pll unavailable if gyro is on standby)
    );
    mpu_write_reg(mpu, MPU_PWR_MGMT_2, 0
        // disable gyro
        | MPU_PWR_MGMT_2_DISABLE_XG
        | MPU_PWR_MGMT_2_DISABLE_YG
        | MPU_PWR_MGMT_2_DISABLE_ZG
    );
    mpu_write_reg(mpu, MPU_CONFIG, 0
        | (mpu->fifo_mode ? MPU_CONFIG_FIFO_MODE : 0)
        | MPU_CONFIG_EXT_SYNC_SET_NONE
        | MPU_CONFIG_DLPF_CFG_0
    );
    mpu_write_reg(mpu, MPU_ACCEL_CONFIG, 0
        | mpu_accel_ranges[mpu->accel_range].flag
    );
    mpu_write_reg(mpu, MPU_ACCEL_CONFIG_2, 0
        | MPU_ACCEL_CONFIG_2_FIFO_SIZE_4096_B
        | MPU_ACCEL_CONFIG_2_ACCEL_FCHOICE_0
        | MPU_ACCEL_CONFIG_2_A_DLPF_CFG_0
    );
    mpu_write_reg(mpu, MPU_SMPLRT_DIV, 0); // divide sample frequency by (SMPLRT_DIV+1)
    mpu_write_reg(mpu, MPU_FIFO_EN, 0
        // | MPU_FIFO_EN_GYROX
        // | MPU_FIFO_EN_GYROY
        // | MPU_FIFO_EN_GYROZ
        | MPU_FIFO_EN_ACCEL
    );
    mpu_write_reg(mpu, MPU_I2C_MST_CTRL, 0
    );
    mpu_write_reg(mpu, MPU_INT_ENABLE, 0
    );
    mpu_write_reg(mpu, MPU_USER_CTRL, 0
        // | MPU_USER_CTRL_DMP_EN
        | MPU_USER_CTRL_FIFO_EN
        // | MPU_USER_CTRL_I2C_MST_EN
        | MPU_USER_CTRL_FIFO_RST
    );

    mpu->fifo_read_total = 0;
    memset(&mpu->fifo_read_time, 0, sizeof(mpu->fifo_read_time));

    /* Emulate reinit_completion here */
    mpu->poll_completion.done = 0;
    mpu->poll_thread = kthread_run(mpu_poll_thread_fn, mpu, "mpu_poll");
    if(IS_ERR(mpu->poll_thread)) {
        dev_err(MPU_DEV(mpu), "poll thread creation failed: %ld", PTR_ERR(mpu->poll_thread));
        mpu->poll_thread = NULL;
        goto fail_pollthread;
    }
    spin_lock(&mpu->poll_wq.lock);
        mpu->poll_wpos = 0;
        mpu->poll_size = 0;
        wake_up_locked(&mpu->poll_wq);
    spin_unlock(&mpu->poll_wq.lock);

    return 0;

    mpu->poll_thread = NULL;
fail_pollthread:

fail_whoami:

    regulator_disable(mpu->vcc_i2c);
fail_vcc_i2c:

    regulator_disable(mpu->vdd_ana);
fail_vdd_ana:

    mpu->powered = false;
    return ret;
}

static void mpu_poweroff(struct mpu_devdata *mpu)
{
    kthread_stop(mpu->poll_thread);
    dev_dbg(MPU_DEV(mpu), "waiting for poll thread to exit...\n");
    wait_for_completion(&mpu->poll_completion);
    dev_dbg(MPU_DEV(mpu), "poll thread exited\n");
    mpu->poll_thread = NULL;

    regulator_disable(mpu->vcc_i2c);
    regulator_disable(mpu->vdd_ana);
    mpu->powered = false;
    wake_up_interruptible(&mpu->poll_wq);
}

/* Buffering */
static int mpu_poll_thread_fn(void *data)
{
    struct mpu_devdata *mpu = data;
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 3 };
    int count;
    int ret;

    /* Enter hard real-time mode */
    sched_setscheduler(current, SCHED_FIFO, &param);

    dev_dbg(MPU_DEV(mpu), "mpu_poll_thread_fn started\n");

    while(!kthread_should_stop()) {
        ret = mpu_read_be16(mpu, MPU_FIFO_COUNTH);
        if (ret < 0) {
            dev_err(MPU_DEV(mpu), "failed to read fifo count: %d\n", ret);
            msleep(500);
            continue;
        }
        count = rounddown(ret, 6);

        if(count > 3500) {
            /* Probable FIFO overflow */
            dev_err(MPU_DEV(mpu), "poll thread FIFO overflow (count=%d); resetting\n", count);
            mpu_write_reg(mpu, MPU_USER_CTRL, MPU_USER_CTRL_FIFO_EN | MPU_USER_CTRL_FIFO_RST);
            usleep_range(10000, 15000);
            continue;
        }

        /* accumulate statistics */
        mpu->fifo_read_total += count;
        getnstimeofday(&mpu->fifo_read_time);

        while(count > 0) {
            int burst = min3(count, MAX_BURST_SIZE, POLL_FIFO_BYTES - mpu->poll_wpos);

            ret = mpu_read(mpu, MPU_FIFO_R_W, &mpu->poll_fifo[mpu->poll_wpos], burst);
            if(ret < 0) {
                dev_err(MPU_DEV(mpu), "failed to read %d bytes from fifo: %d\n", burst, ret);
                break;
            }

            spin_lock(&mpu->poll_wq.lock);
                mpu->poll_wpos += burst;
                if(mpu->poll_wpos >= POLL_FIFO_BYTES)
                    mpu->poll_wpos -= POLL_FIFO_BYTES;
                mpu->poll_size = min(POLL_FIFO_BYTES, mpu->poll_size + burst);
                wake_up_locked(&mpu->poll_wq);
            spin_unlock(&mpu->poll_wq.lock);

            count -= burst;
        }

        usleep_range(20000, 25000);
    }
    dev_dbg(MPU_DEV(mpu), "mpu_poll_thread_fn stopped\n");
    complete(&mpu->poll_completion);

    return 0;
}

/* Device files */
static ssize_t mpu_pwr_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct mpu_devdata *mpu = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", mpu->powered ? 1 : 0);
}

static ssize_t mpu_pwr_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct mpu_devdata *mpu = dev_get_drvdata(dev);
    bool sel_pwr;
    int ret;

    ret = strtobool(buf, &sel_pwr);
    if(ret < 0)
        return ret;

    if(sel_pwr != mpu->powered) {
        if(sel_pwr) {
            ret = mpu_poweron(mpu);
            if(ret < 0)
                return ret;
        } else {
            mpu_poweroff(mpu);
        }
    }

    return size;
}

static ssize_t mpu_accel_range_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct mpu_devdata *mpu = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", mpu_accel_ranges[mpu->accel_range].range);
}

static ssize_t mpu_accel_range_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct mpu_devdata *mpu = dev_get_drvdata(dev);
    unsigned long sel_range;
    int reg;
    int ret;
    int i;

    ret = kstrtoul(buf, 0, &sel_range);
    if(ret < 0)
        return ret;

    for(i=0; i<ARRAY_SIZE(mpu_accel_ranges); i++) {
        if(mpu_accel_ranges[i].range >= sel_range) {
            mpu->accel_range = i;
            goto found;
        }
    }
    return -EINVAL;

found:
    reg = mpu_read_reg(mpu, MPU_ACCEL_CONFIG);
    if(reg < 0)
        return reg;

    ret = mpu_write_reg(mpu, MPU_ACCEL_CONFIG,
        (reg & ~MPU_ACCEL_CONFIG_ACCEL_FS_SEL_MASK) | mpu_accel_ranges[mpu->accel_range].flag);
    if(ret < 0)
        return ret;

    return size;
}

static ssize_t mpu_fifo_read_stat_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct mpu_devdata *mpu = dev_get_drvdata(dev);
    return sprintf(buf, "%llu %ld%09ld\n", mpu->fifo_read_total,
        mpu->fifo_read_time.tv_sec, + mpu->fifo_read_time.tv_nsec);
}

static ssize_t mpu_fifo_read(struct file *filp, struct kobject *kobj,
    struct bin_attribute *attr, char *buffer, loff_t pos_unused, size_t size)
{
    struct mpu_devdata *mpu = dev_get_drvdata(kobj_to_dev(kobj));
    int ret;
    int rpos;

    spin_lock(&mpu->poll_wq.lock);
        ret = wait_event_interruptible_locked(mpu->poll_wq,
            mpu->poll_size >= size || mpu->poll_thread == NULL);

        rpos = mpu->poll_wpos - mpu->poll_size;
        if(rpos < 0)
            rpos += POLL_FIFO_BYTES;

        size = min((int)size, mpu->poll_size);
        mpu->poll_size -= size;
    spin_unlock(&mpu->poll_wq.lock);

    /* If we have anything to return, we can do so even if there's a pending signal */
    if(ret < 0 && size == 0) {
        return ret;
    }

    if(rpos + size > POLL_FIFO_BYTES) {
        /* Copy in two parts */
        size_t tail_len = POLL_FIFO_BYTES - rpos;
        memcpy(&buffer[0], &mpu->poll_fifo[rpos], tail_len);
        memcpy(&buffer[tail_len], &mpu->poll_fifo[0], size - tail_len);
    } else {
        memcpy(buffer, &mpu->poll_fifo[rpos], size);
    }

    return size;
}

#define MPU_DEVICE_ATTR_RO(name) DEVICE_ATTR(name, 0444, mpu_##name##_show, NULL)
#define MPU_DEVICE_ATTR_RW(name) DEVICE_ATTR(name, 0666, mpu_##name##_show, mpu_##name##_store)

static MPU_DEVICE_ATTR_RW(pwr);
static MPU_DEVICE_ATTR_RW(accel_range);
static MPU_DEVICE_ATTR_RO(fifo_read_stat);
static struct device_attribute *device_attrs[] = {
    &dev_attr_pwr,
    &dev_attr_accel_range,
    &dev_attr_fifo_read_stat,
};

static struct bin_attribute device_attr_fifo = {
    .attr = {
        .name = "fifo",
        .mode = 0444,
    },
    .size = 0,
    .read = mpu_fifo_read,
    .write = NULL,
};

static int mpu_create_device_files(struct mpu_devdata *mpu)
{
    int ret;
    int i;

    for (i = 0; i < ARRAY_SIZE(device_attrs); i++) {
        ret = device_create_file(MPU_DEV(mpu), device_attrs[i]);
        if (ret < 0) {
            dev_err(MPU_DEV(mpu), "device file %s could not be created: %d\n",
                device_attrs[i]->attr.name, ret);
            return ret;
        }
    }

    ret = device_create_bin_file(MPU_DEV(mpu), &device_attr_fifo);
    if (ret < 0) {
        dev_err(MPU_DEV(mpu), "bin file 'fifo' could not be created: %d\n", ret);
        return ret;
    }
    return 0;
}

static void mpu_remove_device_files(struct mpu_devdata *mpu)
{
    int i;

    for (i = ARRAY_SIZE(device_attrs) - 1; i >= 0; i--) {
        device_remove_file(MPU_DEV(mpu), device_attrs[i]);
    }

    device_remove_bin_file(MPU_DEV(mpu), &device_attr_fifo);
}

/* Misc device (convenient name to use in the filesystem) */
static const struct file_operations mpu_misc_fops = {
    .owner = THIS_MODULE,
};

static int mpu_register_misc(struct mpu_devdata *mpu)
{
    int ret;

    mpu->miscdev.minor = MISC_DYNAMIC_MINOR;
    mpu->miscdev.name = DEVICE_NAME;
    mpu->miscdev.fops = &mpu_misc_fops;
    mpu->miscdev.parent = &mpu->client->dev;
    mpu->miscdev.mode = 0;

    ret = misc_register(&mpu->miscdev);
    if(ret < 0) {
        /* We don't support registering more than one device simultaneously, sorry */
        dev_err(MPU_DEV(mpu), "failed to register miscdevice: %d\n", ret);
        return ret;
    }

    return 0;
}

static void mpu_deregister_misc(struct mpu_devdata *mpu)
{
    misc_deregister(&mpu->miscdev);
}


/* I2C setup from inv_mpu_core.c */
static int inv_mpu_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mpu_devdata *mpu;
    struct device *dev = &client->dev;
    int ret;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        ret = -ENOSYS;
        dev_err(dev, "i2c_check_functionality failed\n");
        goto fail_before_alloc;
    }

    mpu = kzalloc(sizeof(*mpu), GFP_KERNEL);
    if(!mpu) {
        ret = -ENOMEM;
        dev_err(dev, "memory allocation failed\n");
        goto fail_before_alloc;
    }

    mpu->client = client;
    i2c_set_clientdata(client, mpu);
    mpu->chiptype = id->driver_data;
    mpu->powered = false;

    mpu->accel_range = mpu_accel_range_default;
    mpu->fifo_mode = 0;

    /* initialize sync structures */
    init_waitqueue_head(&mpu->poll_wq);
    init_completion(&mpu->poll_completion);

    ret = mpu_create_device_files(mpu);
    if(ret < 0) {
        dev_err(dev, "create_dev_files failed: %d\n", ret);
        goto fail_create_dev;
    }

    ret = mpu_register_misc(mpu);
    if(ret < 0) {
        dev_err(dev, "register_misc failed: %d\n", ret);
        goto fail_reg_misc;
    }

#ifdef CONFIG_OF
    mpu->vdd_ana = regulator_get(dev, "inven,vdd_ana");
    if(IS_ERR(mpu->vdd_ana)) {
        ret = PTR_ERR(mpu->vdd_ana);
        dev_err(dev, "failed to get vdd_ana regulator: %d\n", ret);
        goto fail_vdd_ana;
    }
    mpu->vcc_i2c = regulator_get(dev, "inven,vcc_i2c");
    if(IS_ERR(mpu->vcc_i2c)) {
        ret = PTR_ERR(mpu->vcc_i2c);
        dev_err(dev, "failed to get vcc_i2c regulator: %d\n", ret);
        goto fail_vcc_i2c;
    }
#endif

    ret = mpu_poweron(mpu);
    if(ret < 0) {
        dev_err(dev, "poweron failed: %d\n", ret);
        goto fail_late;
    }

    dev_info(dev, "ready!\n");

    return 0;

    /* Cleanup */
fail_late:

#ifdef CONFIG_OF
    regulator_put(mpu->vcc_i2c);
fail_vcc_i2c:

    regulator_put(mpu->vdd_ana);
fail_vdd_ana:
#endif

    mpu_deregister_misc(mpu);
fail_reg_misc:

    mpu_remove_device_files(mpu);
fail_create_dev:

    kfree(mpu);
fail_before_alloc:

    return ret;
}

static int inv_mpu_remove(struct i2c_client *client)
{
    struct mpu_devdata *mpu = i2c_get_clientdata(client);

    mpu_poweroff(mpu);
    regulator_put(mpu->vdd_ana);
    regulator_put(mpu->vcc_i2c);
    mpu_deregister_misc(mpu);
    mpu_remove_device_files(mpu);
    kfree(mpu);

    dev_info(&client->dev, "removed.\n");

    return 0;
}

#ifdef CONFIG_PM
static int inv_mpu_resume(struct device *dev)
{
    struct mpu_devdata *mpu = i2c_get_clientdata(to_i2c_client(dev));

    return mpu_poweron(mpu);
}

static int inv_mpu_suspend(struct device *dev)
{
    struct mpu_devdata *mpu = i2c_get_clientdata(to_i2c_client(dev));

    mpu_poweroff(mpu);

    return 0;
}

static const struct dev_pm_ops inv_mpu_pmops = {
    .suspend       = inv_mpu_suspend,
    .resume        = inv_mpu_resume,
};
#define INV_MPU_PMOPS (&inv_mpu_pmops)
#else
#define INV_MPU_PMOPS NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id inv_mpu_id[] = {
    {"itg3500", INV_ITG3500},
    {"mpu3050", INV_MPU3050},
    {"mpu6050", INV_MPU6050},
    {"mpu9150", INV_MPU9150},
    {"mpu6500", INV_MPU6500},
    {"mpu9250", INV_MPU9250},
    {"mpu6xxx", INV_MPU6XXX},
    {"mpu9350", INV_MPU9350},
    {"mpu6515", INV_MPU6515},
    {}
};
MODULE_DEVICE_TABLE(i2c, inv_mpu_id);

static struct i2c_driver inv_mpu_driver = {
    .probe = inv_mpu_probe,
    .remove = inv_mpu_remove,
    .id_table = inv_mpu_id,
    .driver = {
        .owner = THIS_MODULE,
        .name = DEVICE_NAME,
        .pm = INV_MPU_PMOPS,
    },
};

static int __init fastacc_init(void)
{
    return i2c_add_driver(&inv_mpu_driver);
}

static void __exit fastacc_exit(void)
{
    i2c_del_driver(&inv_mpu_driver);
}

module_init(fastacc_init);
module_exit(fastacc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Xiao <brx@cs.cmu.edu>");
MODULE_DESCRIPTION("High-speed accelerometer driver for the Invensense MPU-6xxx and MPU-9xxx devices");
