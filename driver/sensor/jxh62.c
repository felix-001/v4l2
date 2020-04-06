/**
* @file sensor/jxh62.c
* @author rigensen
* @brief 
* @date 三  4/ 1 15:31:54 2020
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <soc/gpio.h>
#include <media/v4l2-subdev.h>

#define JXH62_CHIP_ID_H	(0xa0)
#define JXH62_CHIP_ID_L	(0x62)

#define JXH62_REG_END		0xff
#define JXH62_REG_DELAY		0xfe

#define JXH62_SUPPORT_PCLK (36*1000*1000)
#define SENSOR_OUTPUT_MAX_FPS 30
#define SENSOR_OUTPUT_MIN_FPS 5
#define DRIVE_CAPABILITY_1

#define SENSOR_NAME "jxh62"
#define SENSOR_WIDTH 1280
#define SENSOR_HEIGHT 720

#define log(fmt, args...) printk("%s:%d $ "fmt"\n", __func__, __LINE__, ##args)

static const struct i2c_device_id jxh62_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, jxh62_id);

typedef struct {
    struct v4l2_subdev sd;
} isp_sensor_t;

static int jxh62_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *cc)
{
    cc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cc->bounds.left = 0;
    cc->bounds.top = 0;
    cc->bounds.width = 1280;
    cc->bounds.height = 720;
    cc->defrect.left = 0;
    cc->defrect.top = 0;
    cc->defrect.width = 1280;
    cc->defrect.height = 720;
    cc->pixelaspect.numerator = 1;
    cc->pixelaspect.denominator = 1;
    return 0;
}

static int jxh62_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
    // FIXME need to call apical_command() to set left,top,width, height
    // see isp_core_frame_channel_set_crop()
    // this need to do in framework
    return 0;
}

// sensor 的信息通过这个接口告诉framework
// .vidioc_g_fmt_vid_cap这个接口要使用到这个接口
static int jx62_g_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    fmt->width = SENSOR_WIDTH;
    fmt->height = SENSOR_HEIGHT;
    fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;
    fmt->field = V4L2_FIELD_NONE;
    fmt->colorspace = V4L2_COLORSPACE_SRGB;
    return 0;
}

static const struct v4l2_subdev_video_ops jxh62_video_ops = {
	.cropcap = jxh62_cropcap,
    .s_crop = jxh62_s_crop,
    .g_mbus_fmt = jx62_g_mbus_fmt,
};

static const struct v4l2_subdev_ops jxh62_ops = {
	.video = &jxh62_video_ops,
};

static int jxh62_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
    isp_sensor_t *sensor;

	sensor = (isp_sensor_t *)kzalloc(sizeof(*sensor), GFP_KERNEL);
	if(!sensor){
		log("Failed to allocate sensor subdev.\n");
		return -ENOMEM;
	}
	memset(sensor, 0 ,sizeof(*sensor));

    sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &jxh62_ops);
	v4l2_set_subdev_hostdata(sd, sensor);
    return 0;
}

static int jxh62_remove(struct i2c_client *client)
{
    return 0;
}

static struct i2c_driver jxh62_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.probe		= jxh62_probe,
	.remove		= jxh62_remove,
	.id_table	= jxh62_id,
};

static __init int init_jxh62(void)
{
	return i2c_add_driver(&jxh62_driver);
}

static __exit void exit_jxh62(void)
{
	i2c_del_driver(&jxh62_driver);
}

module_init(init_jxh62);
module_exit(exit_jxh62);

MODULE_DESCRIPTION("A low-level driver for SOI jxh62 sensors");
MODULE_LICENSE("GPL");

