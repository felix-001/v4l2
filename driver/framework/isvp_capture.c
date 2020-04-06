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

#define ISP_VIC_NAME "tx-isp-vic"
#define ISP_CORE_NAME "tx-isp-core"
#define log(fmt, args...) printk("%s:%d $ "fmt"\n", __func__, __LINE__, ##args)

typedef struct {
    struct v4l2_device v4l2_dev;
    struct video_device *vfd;
    struct v4l2_subdev *sensor_sd;
    struct device *dev;
} isp_device_t;

struct isp_core_dev {
    struct v4l2_subdev sd;
};

struct isp_vic_dev {
    struct v4l2_subdev sd;
};

static int isvp_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
    isp_device_t *ispdev = video_drvdata(file);

	strcpy(cap->driver, "isp capture");
	strcpy(cap->card, "isp capture dev");
	strlcpy(cap->bus_info, ispdev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = 0x00000001;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

    return 0;
}

int isvp_s_crop(struct file *file, void *fh, const struct v4l2_crop *a)
{
    isp_device_t *ispdev = video_drvdata(file);

    v4l2_device_call_all(&ispdev->v4l2_dev, 0, video, s_crop, a);
    return 0;
}

static int isvp_cropcap(struct file *file, void *priv, struct v4l2_cropcap *a)
{
    isp_device_t *ispdev = video_drvdata(file);

    v4l2_device_call_all(&ispdev->v4l2_dev, 0, video, cropcap, a);
    return 0;
}

static int isvp_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
    int err;
    isp_device_t *ispdev = video_drvdata(file);
    struct v4l2_mbus_framefmt fmt;
    struct v4l2_pix_format *pix = &f->fmt.pix;
    int depth;

	err = v4l2_subdev_call(ispdev->sensor_sd, video, g_mbus_fmt, &fmt);
    if (err) {
        log("mbus fmt error");
        goto exit;
    }
    f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    pix->width = fmt.width;
    pix->height = fmt.height;
    pix->field = fmt.field;
    pix->colorspace = fmt.colorspace;
    if (fmt.code == V4L2_MBUS_FMT_SBGGR10_1X10) {
        pix->pixelformat = V4L2_PIX_FMT_SBGGR12;
        depth = 16;
    }
    pix->bytesperline = pix->width * depth/8;
    pix->sizeimage = pix->bytesperline * pix->height;

    return 0;
exit:
    return err;
}

static const struct v4l2_ioctl_ops isvp_ioctl_ops = {
    .vidioc_querycap        = isvp_querycap,
    .vidioc_cropcap         = isvp_cropcap,
    .vidioc_s_crop          = isvp_s_crop,
    .vidioc_g_fmt_vid_cap   = isvp_g_fmt,
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

int isp_core_register(struct platform_device *pdev, struct v4l2_device *v4l2_dev)
{
    struct isp_core_dev *isp_core;
    int ret;
    struct v4l2_subdev *sd;

    isp_core = (struct  isp_core_dev*)kzalloc(sizeof(*isp_core), GFP_KERNEL);
	if(!isp_core){
		log("Failed to allocate sensor device\n");
		ret = -ENOMEM;
		goto exit;
	}
    sd = &isp_core->sd;
    v4l2_set_subdevdata(sd, isp_core);
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0){
		log("Failed to register csi-subdev!\n");
		ret = -1;// FIXME
		goto exit;
	}

    return 0;
exit:
    return ret;
}

int vic_dev_register(struct platform_device *pdev, struct v4l2_device *v4l2_dev)
{
    struct isp_vic_dev *vic;
    int ret;
    struct v4l2_subdev *sd;

    vic = (struct isp_vic_dev*)kzalloc(sizeof(*vic), GFP_KERNEL);
	if(!vic){
		log("Failed to allocate sensor device\n");
		ret = -ENOMEM;
		goto exit;
	}
    sd = &vic->sd;
    v4l2_set_subdevdata(sd, vic);
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0){
		log("Failed to register csi-subdev!\n");
		ret = -1;// FIXME
		goto exit;
	}

    return 0;
exit:
    return ret;
    return 0;
}

static int isp_subdev_match(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	isp_device_t* ispdev = (isp_device_t*)data;
	int ret = 0;

	if(!get_device(dev))
		return -ENODEV;
	if(!strncmp(pdev->name, "tx-isp", 6)){
		if(!strcmp(pdev->name, ISP_CORE_NAME))
			ret = isp_core_register(pdev, &ispdev->v4l2_dev);
		else if(!strcmp(pdev->name, ISP_VIC_NAME))
			ret = vic_dev_register(pdev, &ispdev->v4l2_dev);
		else
			log("register all isp device successfully!\n");
	}
	put_device(dev);
	return ret;
}

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

    ispdev->dev = &pdev->dev;
    sprintf(ispdev->v4l2_dev.name, "isp v4l2 name");
    err = v4l2_device_register(ispdev->dev, &ispdev->v4l2_dev);
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
    video_set_drvdata(ispdev->vfd, ispdev);
    adapter = i2c_get_adapter(0);
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
    ispdev->sensor_sd = sd;
	err = v4l2_subdev_call(sd, core, g_chip_ident, NULL);
	if(err) {
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct i2c_adapter *adapter = client->adapter;

        if (adapter)
            i2c_put_adapter(adapter);
        v4l2_device_unregister_subdev(sd);
        goto exit;
	}

	err = bus_for_each_dev(&platform_bus_type, NULL, ispdev, isp_subdev_match);
	if (err) {
		log("Failed to register isp's subdev\n");
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
	isp_device_t* ispdev = platform_get_drvdata(pdev);

    log("isp remove");

	platform_set_drvdata(pdev, NULL);
	v4l2_device_unregister(&ispdev->v4l2_dev);
	kfree(ispdev);
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

