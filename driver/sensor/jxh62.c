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
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
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

static struct regval_list jxh62_init_regs_1280_720_25fps[] = {
	{0x12,0x40},
	{0x0E,0x11},
	{0x0F,0x00},
	{0x10,0x18},
	{0x11,0x80},
	{0x19,0x68},
	{0x20,0x40},
	{0x21,0x06},
	{0x22,0x84},
	{0x23,0x03},
	{0x24,0x00},
	{0x25,0xD0},
	{0x26,0x25},
	{0x27,0x10},
	{0x28,0x15},
	{0x29,0x02},
	{0x2A,0x01},
	{0x2B,0x21},
	{0x2C,0x08},
	{0x2D,0x01},
	{0x2E,0xBB},
	{0x2F,0xC0},
	{0x41,0x88},
	{0x42,0x12},
	{0x39,0x90},
	{0x1D,0xFF},
	{0x1E,0x9F},
	{0x7A,0x80},
	{0x1F,0x20},
	{0x30,0x90},
	{0x31,0x0C},
	{0x32,0xFF},
	{0x33,0x0C},
	{0x34,0x4B},
	{0x35,0xE3},
	{0x36,0x0A},
	{0x38,0x40},
	{0x3A,0x08},
	{0x56,0x02},
	{0x60,0x01},
#ifdef	DRIVE_CAPABILITY_1
	{0x0D,0x50},
#elif defined(DRIVE_CAPABILITY_2)
	{0x0D,0x5c},
#endif
	{0x57,0x80},
	{0x58,0x33},
	{0x5A,0x04},
	{0x5B,0xB6},
	{0x5C,0x08},
	{0x5D,0x67},
	{0x5E,0x04},
	{0x5F,0x08},
	{0x66,0x28},
	{0x67,0xF8},
	{0x68,0x04},
	{0x69,0x74},
	{0x6A,0x1F},
	{0x63,0x82},
	{0x6C,0xC0},
	{0x6E,0x5C},
	{0x82,0x01},
	{0x0C,0x00},
	{0x46,0xC2},
	{0x48,0x7E},
	{0x62,0x40},
	{0x7D,0x57},
	{0x7E,0x28},
	{0x80,0x00},
	{0x4A,0x05},
	{0x49,0x10},
	{0x13,0x81},
	{0x59,0x97},
	{0x12,0x00},
	{0x47,0x47},
	{JXH62_REG_DELAY,250},
	{JXH62_REG_DELAY,250},
	{0x47,0x44},
	{0x1F,0x21},
	{JXH62_REG_END, 0x00},	/* END MARKER */
};

static const struct v4l2_subdev_video_ops jxh62_video_ops = {
	.cropcap = jxh62_cropcap,
    .s_crop = jxh62_s_crop,
    .g_mbus_fmt = jx62_g_mbus_fmt,
};

int jxh62_write(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int jxh62_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != JXH62_REG_END) {
		if (vals->reg_num == JXH62_REG_DELAY) {
			if (vals->value >= (1000 / HZ))
				msleep(vals->value);
			else
				msleep(vals->value);
		} else {
			ret = jxh62_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}

int jxh62_core_ops_init(struct v4l2_subdev *sd, u32 val)
{
    int ret;

    ret = jxh62_write_array(sd, jxh62_init_regs_1280_720_25fps);
    if (ret)
        return ret;
    return 0;
}

struct v4l2_subdev_core_ops jxh62_core_ops= {
    .init = jxh62_core_ops_init, 
};

static const struct v4l2_subdev_ops jxh62_ops = {
	.video = &jxh62_video_ops,
    .core = &jxh62_core_ops,
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

