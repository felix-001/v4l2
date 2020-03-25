/**
 * @file isvp_capture.c
 * @author rigensen
 * @brief 
 * @date 三  3/25 21:35:00 2020
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#define log(fmt, args...) printk("%s:%d $ "fmt"\n", __func__, __LINE__, ##args)

typedef struct {
    struct v4l2_device v4l2_dev;
    struct video_device *vfd;
} isp_device_t;

static int isvp_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
    return 0;
}

static int isvp_g_priority(struct file *file, void *priv, enum v4l2_priority *prio)
{
    return 0;
}

static int isvp_s_priority(struct file *file, void *priv, enum v4l2_priority p)
{
    return 0;
}


static const struct v4l2_ioctl_ops isvp_ioctl_ops = {
    .vidioc_querycap        	= isvp_querycap,
    .vidioc_g_priority		= isvp_g_priority,
    .vidioc_s_priority		= isvp_s_priority,
#if 0
    .vidioc_enum_fmt_vid_cap	= isvp_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap  		= isvp_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap		= isvp_s_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap		= isvp_try_fmt_vid_cap,
    .vidioc_enum_input		= isvp_enum_input,
    .vidioc_s_input			= isvp_s_input,
    .vidioc_g_input			= isvp_g_input,
    .vidioc_reqbufs         	= isvp_reqbufs,
    .vidioc_querybuf        	= isvp_querybuf,
    .vidioc_querystd		= isvp_querystd,
    .vidioc_s_std           	= isvp_s_std,
    .vidioc_g_std			= isvp_g_std,
    .vidioc_qbuf            	= isvp_qbuf,
    .vidioc_dqbuf           	= isvp_dqbuf,
    .vidioc_streamon        	= isvp_streamon,
    .vidioc_streamoff       	= isvp_streamoff,
    .vidioc_cropcap         	= isvp_cropcap,
    .vidioc_enum_dv_timings         = isvp_enum_dv_timings,
    .vidioc_query_dv_timings        = isvp_query_dv_timings,
    .vidioc_s_dv_timings            = isvp_s_dv_timings,
    .vidioc_g_dv_timings            = isvp_g_dv_timings,
    .vidioc_g_chip_ident		= isvp_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
    .vidioc_g_register		= isvp_dbg_g_register,
    .vidioc_s_register		= isvp_dbg_s_register,
#endif
    .vidioc_log_status		= isvp_log_status,
#endif
};

static int isvp_open(struct file *file)
{
    return 0;
}

static int isvp_close(struct file *file)
{
    return 0;
}

static int isvp_mmap(struct file *file, struct vm_area_struct *vma)
{
    return 0;
}

static unsigned int isvp_poll(struct file *file, struct poll_table_struct *wait)
{
    return 0;
}

static struct v4l2_file_operations isvp_fops = {
    .owner = THIS_MODULE,
    .open = isvp_open,
    .release = isvp_close,
    .unlocked_ioctl = video_ioctl2,
    .mmap = isvp_mmap,
    .poll = isvp_poll
};

static struct video_device isvp_video_template = {
    .name		= "isvp-capture",
    .fops		= &isvp_fops,
    .minor		= -1,
    .ioctl_ops	= &isvp_ioctl_ops,
    .release = video_device_release,
};

static int isp_probe(struct platform_device *pdev)
{
    int err;
    isp_device_t *ispdev;
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info;
    struct v4l2_subdev *sd;

    log("isp probe");
    ispdev = (isp_device_t*)kzalloc(sizeof(*ispdev), GFP_KERNEL);
    if (!ispdev) {
        log("Failed to allocate camera device\n");
        err = -ENOMEM;
        goto exit;
    }

    err = v4l2_device_register(&pdev->dev, &ispdev->v4l2_dev);
    if (err) {
        log("Error registering v4l2 device");
        goto exit;
    }
    ispdev->vfd = video_device_alloc();
    if (!ispdev->vfd) {
        log("alloc video device error");
        err = -ENOMEM;
        goto exit;
    }
    *ispdev->vfd = isvp_video_template;
    err = video_register_device(ispdev->vfd, VFL_TYPE_GRABBER, -1);
    if (err) {
        log("Failed to register video device");
        goto free_video_device;
    }
    adapter = i2c_get_adapter(1);
    if (!adapter) {
        log( "Failed to get I2C adapter 1, deferring probe");
        err = -3;// FIXME
        goto exit;
    }
    memset(&board_info, 0, sizeof(board_info));
    memcpy(&board_info.type, "jxh62", I2C_NAME_SIZE);// FIXME: do not hard code `jxh62`
    board_info.addr = 0x30;
    sd = v4l2_i2c_new_subdev_board(&ispdev->v4l2_dev, adapter, &board_info, NULL);
    if (IS_ERR_OR_NULL(sd)) {
        i2c_put_adapter(adapter);
        log( "Failed to acquire subdev jxh62, deferring probe");
        err = -4;// FIXME
        goto exit;
    }
	err = v4l2_subdev_call(sd, core, g_chip_ident, NULL);
	if(err) {
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct i2c_adapter *adapter = client->adapter;

        if (adapter)
            i2c_put_adapter(adapter);
        v4l2_device_unregister_subdev(sd);
        goto exit;
	}

    return 0;
free_video_device:
    video_device_release(ispdev->vfd);
exit:
    return err;
}

static int __exit isp_remove(struct platform_device *pdev)
{
    log("isp remove");
    return 0;
}

static struct platform_driver isp_driver = {
    .probe = isp_probe,
    .remove = __exit_p(isp_remove),
    .driver = {
        .name = "tx-isp",
        .owner = THIS_MODULE,
    },
};

static int __init isp_init(void)
{
    return platform_driver_register(&isp_driver);
}

static void __exit isp_exit(void)
{
    platform_driver_unregister(&isp_driver);
}

module_init(isp_init);
module_exit(isp_exit);

MODULE_AUTHOR("swayinwind");
MODULE_DESCRIPTION("tx isp driver");
MODULE_LICENSE("GPL");

