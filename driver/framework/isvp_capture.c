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
#include <apical-isp/apical_math.h>
#include "apical-isp/system_i2c.h"
#include <apical-isp/apical_isp_io.h>
#include <apical-isp/system_io.h>
#include <apical-isp/apical_configuration.h>
#include <apical-isp/system_interrupts.h>
#include <apical-isp/apical_isp_config.h>
#include "apical-isp/system_semaphore.h"
#include "apical-isp/apical_command_api.h"
#include <apical-isp/apical_isp_core_nomem_settings.h>
#include "apical-isp/sensor_drv.h"
#include <apical-isp/apical_firmware_config.h>
#include "apical-isp/apical_cmd_interface.h"
#include "apical-isp/apical_scaler_lut.h"
#include "apical-isp/sensor_drv.h"
#include <apical-isp/apical_firmware_config.h>
#include <linux/interrupt.h>
#include <asm/irq.h>

#define ISP_VIC_NAME "tx-isp-vic"
#define ISP_CORE_NAME "tx-isp-core"
#define log(fmt, args...) printk("%s:%d $ "fmt"\n", __func__, __LINE__, ##args)
#define ARRSZ(arr) sizeof(arr)/sizeof(arr[0])
#define ISP_CLK_1080P_MODE 90000000
#define isp_readl(base, reg)		__raw_readl((base) + (reg))
#define isp_writel(base, reg, value)		__raw_writel((value), ((base) + (reg)))
#define isp_readw(base, reg)		__raw_readw((base) + (reg))
#define isp_writew(base, reg, value)		__raw_writew((value), ((base) + (reg)))
#define isp_readb(base, reg)		__raw_readb((base) + (reg))
#define isp_writeb(base, reg, value)		__raw_writeb((value), ((base) + (reg)))

#define ISP_TOP_IRQ_CNT		0x0
#define ISP_TOP_IRQ_CNT1		0x20
#define ISP_TOP_IRQ_CNT2		0x24
#define ISP_TOP_IRQ_CLR_1		0x4
#define ISP_TOP_IRQ_CLR_ALL		0x8
#define ISP_TOP_IRQ_STA		0xC
#define ISP_TOP_IRQ_OVF		0x10
#define ISP_TOP_IRQ_ENABLE		0x14
#define ISP_TOP_IRQ_MASK		0x1c
#define ISP_TOP_IRQ_ISP		0xffff
#define ISP_TOP_IRQ_VIC		0x7f0000
#define ISP_TOP_IRQ_ALL		0x7fffff

#define VIC_DB_CFG		         0x0
#define DVP_DATA_POS			(1<<24)
#define DVP_RGB_ORDER			(1<<21)
#define DVP_RAW_ALIG			(1<<20)
#define DVP_DATA_TYPE			(17)
#define DVP_RAW8			(0<<DVP_DATA_TYPE)
#define DVP_RAW10			(1<<DVP_DATA_TYPE)
#define DVP_RAW12			(2<<DVP_DATA_TYPE)
#define DVP_YUV422_16BIT		(3<<DVP_DATA_TYPE)
#define DVP_RGB565_16BIT		(4<<DVP_DATA_TYPE)
#define DVP_BRG565_16BIT		(5<<DVP_DATA_TYPE)
#define DVP_YUV422_8BIT			(6<<DVP_DATA_TYPE)
#define DVP_RGB565_8BIT			(7<<DVP_DATA_TYPE)
#define VIC_RESOLUTION	 	        0x20
#define H_RESOLUTION			(1<<16)
#define V_RESOLUTION			(1)
#define VIC_GLOBAL_CFG             	(0x30)
#define ISP_PRESET_MODE2		(0<<5)
#define ISP_PRESET_MODE3		(1<<5)
#define ISP_PRESET_MODE1		(2<<5)
#define VCKE_EN				(1<<4)
#define BLANK_EN			(2)
#define AB_MODE_SELECT			(0)
#define VIC_CONTROL			(0x34)
#define VIC_RESET			(1<<4)
#define GLB_SAFE_RST			(1<<3)
#define GLB_RST				(1<<2)
#define REG_ENABLE			(1<<1)
#define VIC_SRART			(1<<0)


static system_interrupt_handler_t isr_func[APICAL_IRQ_COUNT] = {NULL};
static void* isr_param[APICAL_IRQ_COUNT] = {NULL};
struct tx_isp_sensor_attribute sensor_attr, *attr = &sensor_attr;
system_tab stab;

/* apical ISP channel define */
enum isp_video_channel_define{
	ISP_FR_VIDEO_CHANNEL,
	ISP_DS1_VIDEO_CHANNEL,
	ISP_DS2_VIDEO_CHANNEL,
	ISP_MAX_OUTPUT_VIDEOS,
};

typedef struct {
    struct v4l2_device v4l2_dev;
    struct video_device *vfd;
    struct v4l2_subdev *sensor_sd;
    struct device *dev;
    struct v4l2_mbus_framefmt mbus;
    int depth;
    void __iomem *irqbase;
    int irq;
} isp_device_t;

struct apical_control {
    uint8_t id;
    uint32_t value;
};

struct isp_core_dev {
    struct v4l2_subdev sd;
    struct task_struct *process_thread;
    struct platform_device *pdev;
    isp_device_t *ispdev;
};

struct isp_vic_dev {
    struct v4l2_subdev sd;
    void __iomem *irqbase;
    void __iomem *portbase;
    int irq;
    spinlock_t slock;
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

    v4l2_subdev_call(ispdev->sensor_sd, video, cropcap, a);
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
        ispdev->depth = depth;
    }
    pix->bytesperline = pix->width * depth/8;
    pix->sizeimage = pix->bytesperline * pix->height;
    ispdev->mbus = fmt;

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

static int isp_core_clk_enable(struct platform_device *pdev)
{
    struct tx_isp_subdev_platform_data *pdata = pdev->dev.platform_data;
    int i;
    struct clk **clks = NULL;
    struct tx_isp_subdev_clk_info *info = pdata->clks;
    int ret = 0;

    for (i=0; i<pdata->clk_num; i++) {
		clks[i] = clk_get(&pdev->dev, info[i].name);
		if (IS_ERR(clks[i])) {
			log("Failed to get %s clock %ld\n", info[i].name, PTR_ERR(clks[i]));
			ret = PTR_ERR(clks[i]);
            goto exit;
		}
		if(info[i].rate != DUMMY_CLOCK_RATE) {
			ret = clk_set_rate(clks[i], ISP_CLK_1080P_MODE);
			if(ret){
				log("Failed to set %s clock rate(%ld)\n", info[i].name, info[i].rate);
                goto exit;
			}
            clk_enable(clks[i]);
		}
    }

exit:
	while(--i >= 0){
		clk_put(clks[i]);
	}
	kfree(clks);
    return ret;
}

static int isp_fw_process(void *data)
{
	while(!kthread_should_stop()){
		apical_process();
		apical_cmd_process();
		apical_connection_process();
	}
	apical_connection_destroy();
	return 0;
}

void system_program_interrupt_event(uint8_t event, uint8_t id)
{
	switch(event)
	{
		case 0: apical_isp_interrupts_interrupt0_source_write(id); break;
		case 1: apical_isp_interrupts_interrupt1_source_write(id); break;
		case 2: apical_isp_interrupts_interrupt2_source_write(id); break;
		case 3: apical_isp_interrupts_interrupt3_source_write(id); break;
		case 4: apical_isp_interrupts_interrupt4_source_write(id); break;
		case 5: apical_isp_interrupts_interrupt5_source_write(id); break;
		case 6: apical_isp_interrupts_interrupt6_source_write(id); break;
		case 7: apical_isp_interrupts_interrupt7_source_write(id); break;
		case 8: apical_isp_interrupts_interrupt8_source_write(id); break;
		case 9: apical_isp_interrupts_interrupt9_source_write(id); break;
		case 10: apical_isp_interrupts_interrupt10_source_write(id); break;
		case 11: apical_isp_interrupts_interrupt11_source_write(id); break;
		case 12: apical_isp_interrupts_interrupt12_source_write(id); break;
		case 13: apical_isp_interrupts_interrupt13_source_write(id); break;
		case 14: apical_isp_interrupts_interrupt14_source_write(id); break;
		case 15: apical_isp_interrupts_interrupt15_source_write(id); break;
	}
}

static int isp_core_ops_init(struct v4l2_subdev *sd, u32 on)
{
    int ret = 0;
    struct isp_core_dev *isp_core = (struct isp_core_dev *)v4l2_get_subdevdata(sd);

    isp_core_clk_enable(isp_core->pdev);
    apical_init();
    apical_connection_init();
    system_program_interrupt_event(APICAL_IRQ_DS1_OUTPUT_END, 50);
    isp_core->process_thread = kthread_run(isp_fw_process, NULL, "apical_isp_fw_process");
    if(IS_ERR_OR_NULL(isp_core->process_thread)){
        log("%s[%d] kthread_run was failed!\n",__func__,__LINE__);
        ret = -1;
        goto exit;
    }

exit:
    return ret;
}

static const struct v4l2_subdev_core_ops isp_core_subdev_core_ops ={
	.init = isp_core_ops_init,
};

int isp_core_video_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
    unsigned int chan = CROP_DS << 16;
    int ret = 0, i;
    struct apical_control controls[] = 
    {
        {IMAGE_RESIZE_WIDTH_ID,     chan + crop->c.width},
        {IMAGE_RESIZE_HEIGHT_ID,    chan + crop->c.height},
        {IMAGE_CROP_XOFFSET_ID,     chan + crop->c.left},
        {IMAGE_CROP_YOFFSET_ID,     chan + crop->c.top},
        {IMAGE_RESIZE_ENABLE_ID,    chan + ENABLE}
    };

    apical_isp_top_bypass_ds1_crop_write(0);
    for (i=0; i<ARRSZ(controls); i++) {
        apical_command(TIMAGE, controls[i].id, controls[i].value, COMMAND_SET, &ret);
    }
    return 0;
}

int isp_core_video_s_stream(struct v4l2_subdev *sd, int enable)
{
    int ret, i;
    isp_device_t *ispdev = container_of(sd->v4l2_dev, isp_device_t, v4l2_dev);

    apical_command(TSYSTEM, ISP_SYSTEM_STATE, PAUSE, COMMAND_SET, &ret);
    /* 2-->module config updates during local vertical blanking */
    apical_isp_top_config_buffer_mode_write(2);
    /* set input port mode, mode1 */
    APICAL_WRITE_32(0x100, 0x00100001);
    apical_isp_top_active_width_write(ispdev->mbus.width);
    apical_isp_top_active_height_write(ispdev->mbus.height);
    if (ispdev->mbus.code == V4L2_MBUS_FMT_SBGGR10_1X10)
        apical_isp_top_rggb_start_write(APICAL_ISP_TOP_RGGB_START_B_GB_GR_R); //Starting color of the rggb pattern
    for(i = 0; i < ARRSZ(apical_downscaler_lut); i++)
        APICAL_WRITE_32(apical_downscaler_lut[i].reg, apical_downscaler_lut[i].value);
	/*
	 * clear interrupts state of isp-core.
	 * Interrupt event clear register writing 0-1 transition will clear the corresponding status bits.
	 */
	apical_isp_interrupts_interrupt_clear_write(0);
	apical_isp_interrupts_interrupt_clear_write(0xffff);
    apical_command(TSYSTEM, ISP_SYSTEM_STATE, RUN, COMMAND_SET, &ret);

    return 0;
}

int isp_core_video_s_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    int ret;
    unsigned int base = 0x00b00 + 0x100 * ISP_DS1_VIDEO_CHANNEL; // the base of address of dma channel write
    isp_device_t *ispdev = container_of(sd->v4l2_dev, isp_device_t, v4l2_dev);

    if (fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10) {
        APICAL_WRITE_32(base, DMA_FORMAT_RAW16 & 0x0f);
        APICAL_WRITE_32(base + 0x04, fmt->height << 16 | fmt->width);
        APICAL_WRITE_32(base + 0x20, fmt->width * (ispdev->depth/8));//lineoffset
        apical_command(TSCENE_MODES, DS1_OUTPUT_MODE_ID, RGB, COMMAND_SET, &ret);
        APICAL_WRITE_32(0x6b0, 0xf);
    }
    return 0;
}

static struct v4l2_subdev_video_ops	isp_core_subdev_video_ops = {
    .s_crop = isp_core_video_s_crop,
    .s_stream = isp_core_video_s_stream,
    .s_mbus_fmt = isp_core_video_s_mbus_fmt,
};

static const struct v4l2_subdev_ops isp_core_ops ={
	.core = &isp_core_subdev_core_ops,
	.video = &isp_core_subdev_video_ops,
};

static int vic_core_ops_init(struct v4l2_subdev *sd, u32 on)
{
	volatile unsigned int reg;
	unsigned long flags;
    struct isp_vic_dev *vic = (struct isp_vic_dev *)v4l2_get_subdevdata(sd);

    reg = isp_readl(vic->irqbase, ISP_TOP_IRQ_ENABLE);
	reg |= on;
	isp_writel(vic->irqbase, ISP_TOP_IRQ_ENABLE, reg);
	spin_lock_irqsave(&vic->slock, flags);
    enable_irq(vic->irq);
    spin_unlock_irqrestore(&vic->slock, flags);

    return 0;
}

int vic_core_video_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
    return 0;
}

int vic_core_video_s_stream(struct v4l2_subdev *sd, int enable)
{
    isp_device_t *ispdev = container_of(sd->v4l2_dev, isp_device_t, v4l2_dev);
    struct v4l2_mbus_framefmt *mbus = &ispdev->mbus;
    struct isp_vic_dev *vic = (struct isp_vic_dev *)v4l2_get_subdevdata(sd);

    if (mbus->code == V4L2_MBUS_FMT_SBGGR10_1X10) {
        int ret;

        isp_writel(vic->portbase, VIC_DB_CFG, DVP_RAW10);
		ret = (mbus->width<< 16) | (mbus->height);
		isp_writel(vic->portbase, VIC_RESOLUTION, ret);
		isp_writel(vic->portbase, VIC_GLOBAL_CFG, ISP_PRESET_MODE1);
		isp_writel(vic->portbase, VIC_CONTROL, REG_ENABLE);
		isp_writel(vic->portbase, VIC_CONTROL, VIC_SRART);
    }
    return 0;
}

static const struct v4l2_subdev_core_ops vic_core_subdev_core_ops ={
	.init = vic_core_ops_init,
};

static struct v4l2_subdev_video_ops	vic_core_subdev_video_ops = {
    .s_crop = vic_core_video_s_crop,
    .s_stream = vic_core_video_s_stream,
};

static const struct v4l2_subdev_ops vic_core_ops ={
	.core = &vic_core_subdev_core_ops,
	.video = &vic_core_subdev_video_ops,
};

int isp_core_register(struct platform_device *pdev, struct v4l2_device *v4l2_dev)
{
    struct isp_core_dev *isp_core;
    int ret;
    struct v4l2_subdev *sd;

    isp_core = (struct isp_core_dev*)kzalloc(sizeof(*isp_core), GFP_KERNEL);
	if(!isp_core){
		log("Failed to allocate sensor device\n");
		ret = -ENOMEM;
		goto exit;
	}
    sd = &isp_core->sd;
    v4l2_subdev_init(sd, &isp_core_ops);
    v4l2_set_subdevdata(sd, isp_core);
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0){
		log("Failed to register isp core!\n");
		ret = -1;// FIXME
		goto exit;
	}
    isp_core->pdev = pdev;

    return 0;
exit:
    return ret;
}

int vic_dev_register(struct platform_device *pdev, struct v4l2_device *v4l2_dev)
{
    struct isp_vic_dev *vic;
    int ret;
    struct v4l2_subdev *sd;
    isp_device_t *ispdev = container_of(v4l2_dev, isp_device_t, v4l2_dev);

    vic = (struct isp_vic_dev*)kzalloc(sizeof(*vic), GFP_KERNEL);
	if(!vic){
		log("Failed to allocate sensor device\n");
		ret = -ENOMEM;
		goto exit;
	}
    sd = &vic->sd;
    vic->irqbase = ispdev->irqbase;
    v4l2_subdev_init(sd, &vic_core_ops);
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

static irqreturn_t isp_irq_thread_handle(int this_irq, void *dev)
{
    irqreturn_t retval = IRQ_HANDLED;

    return retval;
}

static irqreturn_t isp_irq_handle(int this_irq, void *dev)
{
    irqreturn_t retval = IRQ_HANDLED;

    return retval;
}

static int isp_irq_init(isp_device_t *ispdev)
{
	struct platform_device *pdev = to_platform_device(ispdev->dev);
    int irq, err;
    struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || !irq) {
		log("%s[%d] Not enough platform resources",__func__,__LINE__);
		err = -ENODEV;
		goto exit;
	}
	res = request_mem_region(res->start, res->end - res->start + 1, dev_name(&pdev->dev));
	if (!res) {
		log("%s[%d] Not enough memory for resources\n", __func__,__LINE__);
		err = -EBUSY;
		goto exit;
	}

	ispdev->irqbase = ioremap(res->start, res->end - res->start + 1);
	if (!ispdev->irqbase) {
		log("%s[%d] Unable to ioremap registers\n", __func__,__LINE__);
		err = -ENXIO;
		goto exit;
	}
	err = request_threaded_irq(irq, isp_irq_handle, isp_irq_thread_handle, IRQF_ONESHOT, "isp", ispdev);
	if(err){
		log("%s[%d] Failed to request irq(%d).\n", __func__,__LINE__, irq);
		err = -EINTR;
		goto exit;
	}
    ispdev->irq = irq;
    return 0;
exit:
    return err;
}

static int isp_register_sensor(isp_device_t *ispdev)
{
    struct v4l2_subdev *sd;
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info;
    int err;

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
    return 0;
exit:
    return err;
}

static int isp_probe(struct platform_device *pdev)
{
    int err;
    isp_device_t *ispdev;

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
    err = isp_irq_init(ispdev);
    if (err) {
        log("Failed to init irq");
        goto free_video_device;
    }

    err = isp_register_sensor(ispdev);
    if (err) {
        log("Failed to register sensor");
        goto free_video_device;
    }
	err = bus_for_each_dev(&platform_bus_type, NULL, ispdev, isp_subdev_match);
	if (err) {
		log("Failed to register isp's subdev\n");
		goto exit;
	}
    v4l2_device_call_all(&ispdev->v4l2_dev, 0, core, init, 1);

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

void system_set_interrupt_handler(uint8_t source,
		system_interrupt_handler_t handler, void* param)
{
	isr_func[source] = handler;
	isr_param[source] = param;
}

static inline void isp_clear_irq_source(void)
{
	int event = 0;
	for(event = 0; event < APICAL_IRQ_COUNT; event++){
		system_program_interrupt_event(event, 0);
	}
}

void system_init_interrupt(void)
{
	isp_clear_irq_source();
}

// FIXME    
void system_hw_interrupts_disable(void)
{
}

// FIXME
// TODO 
void system_hw_interrupts_enable(void)
{
}

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

