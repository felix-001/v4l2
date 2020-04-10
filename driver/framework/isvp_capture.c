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
#define ISP_DMA_WRITE_MAXBASE_NUM 5
#define TX_ISP_VERSION_SIZE 8
#define TX_ISP_VERSION_ID "1.38"
#define TX_ISP_PRIV_PARAM_FLAG_SIZE	8

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

enum __tx_isp_private_parameters_index {
	TX_ISP_PRIV_PARAM_BASE_INDEX,
	TX_ISP_PRIV_PARAM_CUSTOM_INDEX,
	TX_ISP_PRIV_PARAM_MAX_INDEX,
};

enum __tx_isp_private_parameters_mode {
	TX_ISP_PRIV_PARAM_DAY_MODE,
	TX_ISP_PRIV_PARAM_NIGHT_MODE,
	TX_ISP_PRIV_PARAM_BUTT_MODE,
};

typedef struct {
    struct v4l2_device v4l2_dev;
    struct video_device *vfd;
    struct v4l2_subdev *sensor_sd;
    struct device *dev;
    struct v4l2_mbus_framefmt mbus;
    int depth;
    void __iomem *irqbase;
    spinlock_t slock;
    int irq;
    volatile int state;
} isp_device_t;

static isp_device_t *ispdevp;

struct apical_control {
    uint8_t id;
    uint32_t value;
};

#define ISP_DMA_WRITE_MAXBASE_NUM 5
#define ISP_DMA_WRITE_BANK_FLAG_UNCONFIG 0
#define ISP_DMA_WRITE_BANK_FLAG_CONFIG 1
enum apical_isp_format_check_index {
	APICAL_ISP_INPUT_RAW_FMT_INDEX_START = 0,
	APICAL_ISP_INPUT_RGB888_FMT_INDEX_START = 0,
	APICAL_ISP_INPUT_YUV_FMT_INDEX_START = 0,
	APICAL_ISP_NV12_FMT_INDEX = 0,
	APICAL_ISP_NV21_FMT_INDEX,
	APICAL_ISP_YUYV_FMT_INDEX,
	APICAL_ISP_UYVY_FMT_INDEX,
	APICAL_ISP_INPUT_YUV_FMT_INDEX_END = APICAL_ISP_UYVY_FMT_INDEX,
	APICAL_ISP_YUV444_FMT_INDEX,
	APICAL_ISP_INPUT_RGB565_FMT_INDEX_START = APICAL_ISP_YUV444_FMT_INDEX,
	APICAL_ISP_RGB565_FMT_INDEX,
	APICAL_ISP_INPUT_RGB565_FMT_INDEX_END = APICAL_ISP_RGB565_FMT_INDEX,
	APICAL_ISP_RGB24_FMT_INDEX,
	APICAL_ISP_RGB888_FMT_INDEX,
	APICAL_ISP_INPUT_RGB888_FMT_INDEX_END = APICAL_ISP_RGB888_FMT_INDEX,
	APICAL_ISP_RGB310_FMT_INDEX,
	APICAL_ISP_RAW_FMT_INDEX,
	APICAL_ISP_INPUT_RAW_FMT_INDEX_END = APICAL_ISP_RAW_FMT_INDEX,
	APICAL_ISP_FMT_MAX_INDEX,
};

struct apical_isp_contrl {
	unsigned int infmt;
	unsigned short inwidth;
	unsigned short inheight;
	unsigned int pattern;
	enum apical_isp_format_check_index fmt_start;
	enum apical_isp_format_check_index fmt_end;
};

struct isp_core_dev {
    struct v4l2_subdev sd;
    struct task_struct *process_thread;
    struct platform_device *pdev;
    isp_device_t *ispdev;
    volatile int state;
    unsigned char vflip_state;
    unsigned int hflip_state; //0:disable, 1: enable
	struct list_head fifo;
	unsigned char bank_flag[ISP_DMA_WRITE_MAXBASE_NUM];
	unsigned int banks_addr[ISP_DMA_WRITE_MAXBASE_NUM];
    unsigned char usingbanks;
    volatile unsigned int frame_state; // 0 : idle, 1 : processing
    unsigned char reset_dma_flag;
    unsigned char dma_state;
    unsigned char vflip_flag[ISP_DMA_WRITE_MAXBASE_NUM];
    unsigned int isp_daynight_switch;
    struct apical_isp_contrl contrl;
};

struct isp_vic_dev {
    struct v4l2_subdev sd;
    void __iomem *irqbase;
    void __iomem *portbase;
    int irq;
    spinlock_t slock;
};

struct frame_channel_buffer {
	struct list_head entry;
	unsigned int addr;
	void *priv;
};

#define CONTRAST_CURVES_MAXNUM 10
typedef unsigned char contrast_curves[CONTRAST_CURVES_MAXNUM][2];

typedef struct __tx_isp_private_customer_paramters{
	union {
		struct {
			unsigned int 				: 2;
			unsigned int sensor_offset	: 1;
			unsigned int digital_gain	: 1;
			unsigned int gamma_fe		: 1;
			unsigned int raw_front		: 1;
			unsigned int defect_pixel	: 1;
			unsigned int frame_stitch	: 1;
			unsigned int gamma_fe_pos	: 1;
			unsigned int sinter		: 1;
			unsigned int temper		: 1;
			unsigned int order		: 1;
			unsigned int wb_module	: 1;
			unsigned int 			: 1;
			unsigned int mesh		: 1;
			unsigned int iridix		: 1;
			unsigned int 			: 1;
			unsigned int matrix		: 1;
			unsigned int fr_crop		: 1;
			unsigned int fr_gamma		: 1;
			unsigned int fr_sharpen		: 1;
			unsigned int 			: 3;
			unsigned int ds1_crop		: 1;
			unsigned int ds1_scaler		: 1;
			unsigned int ds1_gamma		: 1;
			unsigned int ds1_sharpen	: 1;
			unsigned int 			: 4;
		};
		unsigned int top;
	};
	/* green equalization */
	unsigned int ge_strength;
	unsigned int ge_threshold;
	unsigned int ge_slope;
	unsigned int ge_sensitivity;
	/* defect pixel correct configuration */
	unsigned int dp_module;
	unsigned int hpdev_threshold;
	unsigned int line_threshold;
	unsigned int hp_blend;
	/* demosaic configuration */
	unsigned int dmsc_vh_slope;
	unsigned int dmsc_aa_slope;
	unsigned int dmsc_va_slope;
	unsigned int dmsc_uu_slope;
	unsigned int dmsc_sat_slope;
	unsigned int dmsc_vh_threshold;
	unsigned int dmsc_aa_threshold;
	unsigned int dmsc_va_threshold;
	unsigned int dmsc_uu_threshold;
	unsigned int dmsc_sat_threshold;
	unsigned int dmsc_vh_offset;
	unsigned int dmsc_aa_offset;
	unsigned int dmsc_va_offset;
	unsigned int dmsc_uu_offset;
	unsigned int dmsc_sat_offset;
	unsigned int dmsc_luminance_thresh;
	unsigned int dmsc_np_offset;
	unsigned int dmsc_config;
	unsigned int dmsc_ac_threshold;
	unsigned int dmsc_ac_slope;
	unsigned int dmsc_ac_offset;
	unsigned int dmsc_fc_slope;
	unsigned int dmsc_fc_alias_slope;
	unsigned int dmsc_fc_alias_thresh;
	struct {
		unsigned int dmsc_np_off : 6;
		unsigned int dmsc_np_reflect : 1;
		unsigned int : 25;
	};
	/* Temper */
	unsigned int temper_recursion_limit;
	/* WDR configuration */
	unsigned int wdr_short_thresh;
	unsigned int wdr_long_thresh;
	unsigned int wdr_expo_ratio_thresh;
	unsigned int wdr_stitch_correct;
	unsigned int wdr_stitch_error_thresh;
	unsigned int wdr_stitch_error_limit;
	unsigned int wdr_stitch_bl_long;
	unsigned int wdr_stitch_bl_short;
	unsigned int wdr_stitch_bl_output;
	/* other configuration */
	unsigned int max_isp_dgain;
	unsigned int max_sensor_again;

	unsigned char sharpness;
	unsigned char saturation;
	unsigned char brightness;
	/* the parameters is contrast curve */
	contrast_curves contrast;
} TXispPrivCustomerParamer;

typedef struct __tx_isp_private_parameters_header{
	char flag[TX_ISP_PRIV_PARAM_FLAG_SIZE];
	unsigned int size;		/* the memory size of the parameter array */
	unsigned int crc;
} TXispPrivParamHeader;

typedef struct __tx_isp_private_parameters_manage {
	char version[TX_ISP_VERSION_SIZE];
	TXispPrivParamHeader headers[TX_ISP_PRIV_PARAM_MAX_INDEX];
	void *data;								//the base address of all data.
	unsigned int data_size;
	void *fw_data;								//the base address of isp FW parameters.
	ApicalCalibrations isp_param[TX_ISP_PRIV_PARAM_BUTT_MODE];			//the struct of private0 manager.
	LookupTable param_table[_CALIBRATION_TOTAL_SIZE * TX_ISP_PRIV_PARAM_BUTT_MODE];
	void *base_buf;							//the address of private0 data.
	TXispPrivCustomerParamer *customer;				//the struct of private1 pointer.
	void *customer_buf;							//the address of private1 data.
} TXispPrivParamManage;

enum {
	ISP_STATE_STOP,
	ISP_STATE_START,
	ISP_STATE_RUN,
};

typedef enum isp_core_mode_day_and_night {
	ISP_CORE_RUNING_MODE_DAY_MODE,
	ISP_CORE_RUNING_MODE_NIGHT_MODE,
	ISP_CORE_RUNING_MODE_BUTT,
} ISP_CORE_MODE_DN_E;

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

static struct frame_channel_buffer *pop_buffer_fifo(struct list_head *fifo)
{
	struct frame_channel_buffer *buf;

	if(!list_empty(fifo)){
		buf = list_first_entry(fifo, struct frame_channel_buffer, entry);
        log("^-^ %s %d %p %p^-^\n",__func__,__LINE__, fifo, buf);
		list_del(&(buf->entry));
	}else
		buf = NULL;

	return buf;
}

static void inline push_buffer_fifo(struct list_head *fifo, struct frame_channel_buffer *buf)
{
    log("^@^ %s %d %p %p^-^\n",__func__,__LINE__, fifo, buf);
	list_add_tail(&(buf->entry), fifo);
}

static void inline cleanup_buffer_fifo(struct list_head *fifo)
{
	struct frame_channel_buffer *buf;

	while(!list_empty(fifo)){
		buf = list_first_entry(fifo, struct frame_channel_buffer, entry);
		list_del(&(buf->entry));
	}
}


static int isp_configure_base_addr(struct isp_core_dev *isp_core)
{
	struct frame_channel_buffer *buf;
	unsigned int hw_dma = 0;
	unsigned char current_bank = 0;
	unsigned char bank_id = 0;
	unsigned char i = 0;
    int bytesperline = 0;
    isp_device_t *ispdev = container_of(isp_core->sd.v4l2_dev, isp_device_t, v4l2_dev);
    struct v4l2_mbus_framefmt *mbus = &ispdev->mbus;
    int sizeimage;

    bytesperline = mbus->width * (ispdev->depth/8);
    sizeimage = bytesperline * mbus->height;
    // TODO need to set isp_core->state
    if(isp_core->state == ISP_STATE_RUN){
        hw_dma = APICAL_READ_32(0xb24 + 0x100*ISP_DS1_VIDEO_CHANNEL);
        current_bank = (hw_dma >> 8) & 0x7;
        /* The begin pointer is next bank. */
        for(i = 0, bank_id = current_bank; i < isp_core->usingbanks; i++, bank_id++){
            bank_id = bank_id % isp_core->usingbanks;
            if(isp_core->bank_flag[bank_id] == 0){
                buf = pop_buffer_fifo(&isp_core->fifo);
                if(buf != NULL){
                    APICAL_WRITE_32((0xb00 + 0x08 + 0x100 * ISP_DS1_VIDEO_CHANNEL + 0x04 * bank_id), buf->addr + sizeimage - bytesperline);// lineoffset
                    isp_core->bank_flag[bank_id] = 1;
                } else
                    break;
            }
        }
    }
	return 0;
}

static int isp_enable_dma_transfer(struct isp_core_dev *isp_core, int onoff)
{
    if(onoff)
        APICAL_WRITE_32(0xb00 + 0x24 + 0x100*(ISP_DS1_VIDEO_CHANNEL), 0x02);// axi_port_enable set 1, frame_write_cancel set 0.
    else
        APICAL_WRITE_32(0xb00 + 0x24 + 0x100*(ISP_DS1_VIDEO_CHANNEL), 0x03);// axi_port_enable set 1, frame_write_cancel set 0.
	return 0;
}

static inline int isp_enable_channel(struct isp_core_dev *isp_core)
{
	unsigned int hw_dma = 0;
	unsigned char next_bank = 0;

	hw_dma = APICAL_READ_32(0xb24 + 0x100 * ISP_DS1_VIDEO_CHANNEL);
	next_bank = (((hw_dma >> 8) & 0x7) + 1) % isp_core->usingbanks;
	if(isp_core->bank_flag[next_bank] ^ isp_core->dma_state){
		isp_core->dma_state = isp_core->bank_flag[next_bank];
		isp_enable_dma_transfer(isp_core, isp_core->dma_state);
	}
	return 0;
}

#define ISP_CHAN_DMA_STAT (1<<16)
#define ISP_CHAN_DMA_ACTIVE (1<<16)
static inline void isp_core_update_addr(struct isp_core_dev *isp_core)
{
	unsigned int y_hw_dma = 0;
	unsigned int uv_hw_dma = 0;
	unsigned char current_bank = 0;
	unsigned char uv_bank = 0;
	unsigned char last_bank = 0;
	unsigned char next_bank = 0;
	unsigned char bank_id = 0;
	unsigned int current_active = 0;
	unsigned int value = 0;
    //isp_device_t *ispdev = container_of(isp_core->sd.v4l2_dev, isp_device_t, v4l2_dev);

    y_hw_dma = APICAL_READ_32(0xb24 + 0x100 * ISP_DS1_VIDEO_CHANNEL);
    current_active |= y_hw_dma;
	current_bank = (y_hw_dma >> 8) & 0x7;
	uv_bank = (uv_hw_dma >> 8) & 0x7;

	if(isp_core->reset_dma_flag){
		log("y_bank = %d, nv_bank = %d\n", current_bank, uv_bank);
		value = (0x0<<3) | (isp_core->usingbanks - 1);
        APICAL_WRITE_32(0xb1c + 0x100*ISP_DS1_VIDEO_CHANNEL, value);
		isp_core->reset_dma_flag = 0;
	}

	if((current_active & ISP_CHAN_DMA_STAT) == ISP_CHAN_DMA_ACTIVE){
		last_bank = (y_hw_dma >> 11) & 0x7;
		bank_id = last_bank;
	} else {
		bank_id = current_bank;
	}
    // TODO  
    // frame_channel_video_irq_notify move to here
	next_bank = (current_bank + 1) % isp_core->usingbanks;
	if(isp_core->bank_flag[next_bank] != 1) {
		isp_enable_channel(isp_core);
	}
	return;
}

static int isp_set_buffer_lineoffset_vflip_disable(struct isp_core_dev *isp_core)
{
    isp_device_t *ispdev = container_of(isp_core->sd.v4l2_dev, isp_device_t, v4l2_dev);
    struct v4l2_mbus_framefmt *mbus = &ispdev->mbus;
    int bytesperline;

    bytesperline = mbus->width * (ispdev->depth/8);
    APICAL_WRITE_32(0xb00 + 0x20 + 0x100 * ISP_DS1_VIDEO_CHANNEL, bytesperline);//lineoffset
	return 0;
}

static int isp_set_buffer_lineoffset_vflip_enable(struct isp_core_dev *isp_core)
{
    isp_device_t *ispdev = container_of(isp_core->sd.v4l2_dev, isp_device_t, v4l2_dev);
    struct v4l2_mbus_framefmt *mbus = &ispdev->mbus;
    int bytesperline;

    bytesperline = mbus->width * (ispdev->depth/8);
    APICAL_WRITE_32(0xb00 + 0x20 + 0x100 * ISP_DS1_VIDEO_CHANNEL, -bytesperline);//lineoffset

	return 0;
}

static int isp_modify_dma_direction(struct isp_core_dev *isp_core)
{
	unsigned int hw_dma = 0;
	unsigned char next_bank = 0;

	if(isp_core->state == ISP_STATE_RUN){
		hw_dma = APICAL_READ_32(0xb24 + 0x100 * ISP_DS1_VIDEO_CHANNEL);
		next_bank = (((hw_dma >> 8) & 0x7) + 1) % isp_core->usingbanks;
		if(isp_core->vflip_flag[next_bank] ^ isp_core->vflip_state){
			isp_core->vflip_state = isp_core->vflip_flag[next_bank];
			if(isp_core->vflip_state){
				isp_set_buffer_lineoffset_vflip_enable(isp_core);
			}else{
				isp_set_buffer_lineoffset_vflip_disable(isp_core);
			}
		}
	}
	return 0;
}

int apical_dynamic_calibration(LookupTable** table )
{
    int ret;
#define APICAL_CALIBRATION(cmd) apical_api_calibration(cmd, COMMAND_SET, \
        table[_##cmd]->ptr, table[_##cmd]->rows * table[_##cmd]->cols * table[_##cmd]->width, &ret)

    APICAL_CALIBRATION(CALIBRATION_NP_LUT_MEAN);
    APICAL_CALIBRATION(CALIBRATION_EVTOLUX_PROBABILITY_ENABLE);
    APICAL_CALIBRATION(CALIBRATION_AE_EXPOSURE_AVG_COEF);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_AVG_COEF);
    APICAL_CALIBRATION(CALIBRATION_AF_MIN_TABLE);
    APICAL_CALIBRATION(CALIBRATION_AF_MAX_TABLE);
    APICAL_CALIBRATION(CALIBRATION_AF_WINDOW_RESIZE_TABLE);
    APICAL_CALIBRATION(CALIBRATION_EXP_RATIO_TABLE);
    APICAL_CALIBRATION(CALIBRATION_CCM_ONE_GAIN_THRESHOLD);
    APICAL_CALIBRATION(CALIBRATION_FLASH_RG);
    APICAL_CALIBRATION(CALIBRATION_FLASH_BG);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_STRENGTH_MAXIMUM_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_STRENGTH_MAXIMUM_WDR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_BLACK_PRC);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_GAIN_MAX);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_MIN_MAX_STR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_EV_LIM_FULL_STR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_EV_LIM_NO_STR_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_EV_LIM_NO_STR_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_AE_CORRECTION_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_AE_CORRECTION_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_AE_EXPOSURE_CORRECTION);
    APICAL_CALIBRATION(CALIBRATION_SINTER_STRENGTH_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_STRENGTH_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_STRENGTH1_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_STRENGTH1_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_THRESH1_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_THRESH1_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_THRESH4_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SINTER_THRESH4_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SHARP_ALT_D_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHARP_ALT_D_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SHARP_ALT_UD_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHARP_ALT_UD_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_SHARPEN_FR_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHARPEN_FR_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHARPEN_DS1_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHARPEN_DS1_WDR);
    APICAL_CALIBRATION(CALIBRATION_DEMOSAIC_NP_OFFSET_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_DEMOSAIC_NP_OFFSET_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_MESH_SHADING_STRENGTH);
    APICAL_CALIBRATION(CALIBRATION_SATURATION_STRENGTH_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_TEMPER_STRENGTH);
    APICAL_CALIBRATION(CALIBRATION_STITCHING_ERROR_THRESH);
    APICAL_CALIBRATION(CALIBRATION_DP_SLOPE_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_DP_SLOPE_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_DP_THRESHOLD_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_DP_THRESHOLD_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_AE_BALANCED_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_AE_BALANCED_WDR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_STRENGTH_TABLE);
    APICAL_CALIBRATION(CALIBRATION_RGB2YUV_CONVERSION);
    APICAL_CALIBRATION(CALIBRATION_EVTOLUX_EV_LUT_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_EVTOLUX_EV_LUT_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_EVTOLUX_LUX_LUT);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_R_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_G_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_B_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_R_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_G_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_B_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_R_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_G_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_B_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_R_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_G_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_A_B_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_R_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_G_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_TL84_B_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_R_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_G_WDR);
    APICAL_CALIBRATION(CALIBRATION_SHADING_LS_D65_B_WDR);
    APICAL_CALIBRATION(CALIBRATION_NOISE_PROFILE_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_DEMOSAIC_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_NOISE_PROFILE_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_DEMOSAIC_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_GAMMA_FE_0_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_GAMMA_FE_1_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_R_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_GR_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_GB_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_B_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_R_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_GR_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_GB_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_BLACK_LEVEL_B_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_GAMMA_LINEAR);
    APICAL_CALIBRATION(CALIBRATION_GAMMA_FS_HDR);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_RGB2REC709);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_REC709TORGB);
    APICAL_CALIBRATION(CALIBRATION_IRIDIX_ASYMMETRY);
    APICAL_CALIBRATION(CALIBRATION_DEFECT_PIXELS);

    return 0;
}

int apical_green_equalization(TXispPrivCustomerParamer *customer)
{
    /* green equalization */
    apical_isp_raw_frontend_ge_strength_write(customer->ge_strength);
    apical_isp_raw_frontend_ge_threshold_write(customer->ge_threshold);
    apical_isp_raw_frontend_ge_slope_write(customer->ge_slope);
    apical_isp_raw_frontend_ge_sens_write(customer->ge_sensitivity);

    /* dpc configuration	 */
    apical_isp_raw_frontend_dp_enable_write(customer->dp_module);
    apical_isp_raw_frontend_hpdev_threshold_write(customer->hpdev_threshold);
    apical_isp_raw_frontend_line_thresh_write(customer->line_threshold);
    apical_isp_raw_frontend_hp_blend_write(customer->hp_blend);

    apical_isp_demosaic_vh_slope_write(customer->dmsc_vh_slope);
    apical_isp_demosaic_aa_slope_write(customer->dmsc_aa_slope);
    apical_isp_demosaic_va_slope_write(customer->dmsc_va_slope);
    apical_isp_demosaic_uu_slope_write(customer->dmsc_uu_slope);
    apical_isp_demosaic_sat_slope_write(customer->dmsc_sat_slope);
    apical_isp_demosaic_vh_thresh_write(customer->dmsc_vh_threshold);
    apical_isp_demosaic_aa_thresh_write(customer->dmsc_aa_threshold);
    apical_isp_demosaic_va_thresh_write(customer->dmsc_va_threshold);
    apical_isp_demosaic_uu_thresh_write(customer->dmsc_uu_threshold);
    apical_isp_demosaic_sat_thresh_write(customer->dmsc_sat_threshold);
    apical_isp_demosaic_vh_offset_write(customer->dmsc_vh_offset);
    apical_isp_demosaic_aa_offset_write(customer->dmsc_aa_offset);
    apical_isp_demosaic_va_offset_write(customer->dmsc_va_offset);
    apical_isp_demosaic_uu_offset_write(customer->dmsc_uu_offset);
    apical_isp_demosaic_sat_offset_write(customer->dmsc_sat_offset);
    apical_isp_demosaic_lum_thresh_write(customer->dmsc_luminance_thresh);
    apical_isp_demosaic_np_offset_write(customer->dmsc_np_offset);
    apical_isp_demosaic_dmsc_config_write(customer->dmsc_config);
    apical_isp_demosaic_ac_thresh_write(customer->dmsc_ac_threshold);
    apical_isp_demosaic_ac_slope_write(customer->dmsc_ac_slope);
    apical_isp_demosaic_ac_offset_write(customer->dmsc_ac_offset);
    apical_isp_demosaic_fc_slope_write(customer->dmsc_fc_slope);
    apical_isp_demosaic_fc_alias_slope_write(customer->dmsc_fc_alias_slope);
    apical_isp_demosaic_fc_alias_thresh_write(customer->dmsc_fc_alias_thresh);
    apical_isp_demosaic_np_off_write(customer->dmsc_np_off);
    apical_isp_demosaic_np_off_reflect_write(customer->dmsc_np_reflect);

    apical_isp_temper_recursion_limit_write(customer->temper_recursion_limit);
    apical_isp_frame_stitch_short_thresh_write(customer->wdr_short_thresh);
    apical_isp_frame_stitch_long_thresh_write(customer->wdr_long_thresh);
    apical_isp_frame_stitch_exposure_ratio_write(customer->wdr_expo_ratio_thresh);
    apical_isp_frame_stitch_stitch_correct_write(customer->wdr_stitch_correct);
    apical_isp_frame_stitch_stitch_error_thresh_write(customer->wdr_stitch_error_thresh);
    apical_isp_frame_stitch_stitch_error_limit_write(customer->wdr_stitch_error_limit);
    apical_isp_frame_stitch_black_level_out_write(customer->wdr_stitch_bl_long);
    apical_isp_frame_stitch_black_level_short_write(customer->wdr_stitch_bl_short);
    apical_isp_frame_stitch_black_level_long_write(customer->wdr_stitch_bl_output);

    return 0;
}

int set_stab(int ret, LookupTable** table)
{ 
    if (reason == IMAGE_WDR_MODE_LINEAR) {
        stab.global_minimum_sinter_strength = *((uint16_t *)(table[ _CALIBRATION_SINTER_STRENGTH_LINEAR]->ptr) + 1);
        stab.global_maximum_sinter_strength = *((uint16_t *)(table[ _CALIBRATION_SINTER_STRENGTH_LINEAR]->ptr) + 
                table[_CALIBRATION_SINTER_STRENGTH_LINEAR]->rows * table[_CALIBRATION_SINTER_STRENGTH_LINEAR]->cols -1 );
        stab.global_maximum_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_D_LINEAR]->ptr) + 1);
        stab.global_minimum_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_D_LINEAR]->ptr) +
                table[_CALIBRATION_SHARP_ALT_D_LINEAR]->rows * table[_CALIBRATION_SHARP_ALT_D_LINEAR]->cols -1 );
        stab.global_maximum_un_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_UD_LINEAR]->ptr) + 1);
        stab.global_minimum_un_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_UD_LINEAR]->ptr) +
                table[_CALIBRATION_SHARP_ALT_UD_LINEAR]->rows * table[_CALIBRATION_SHARP_ALT_UD_LINEAR]->cols -1 );
        stab.global_maximum_iridix_strength = *(uint8_t *)(table[_CALIBRATION_IRIDIX_STRENGTH_MAXIMUM_LINEAR]->ptr);
    } else if (reason == IMAGE_WDR_MODE_FS_HDR) {
        stab.global_minimum_sinter_strength = *((uint16_t *)(table[ _CALIBRATION_SINTER_STRENGTH_FS_HDR]->ptr) + 1);
        stab.global_maximum_sinter_strength = *((uint16_t *)(table[ _CALIBRATION_SINTER_STRENGTH_FS_HDR]->ptr) +
                table[_CALIBRATION_SINTER_STRENGTH_LINEAR]->rows * table[_CALIBRATION_SINTER_STRENGTH_LINEAR]->cols -1 );
        stab.global_maximum_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_D_FS_HDR]->ptr) + 1);
        stab.global_minimum_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_D_FS_HDR]->ptr) +
                table[_CALIBRATION_SHARP_ALT_D_LINEAR]->rows * table[_CALIBRATION_SHARP_ALT_D_LINEAR]->cols -1 );
        stab.global_maximum_un_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_UD_FS_HDR]->ptr) + 1);
        stab.global_minimum_un_directional_sharpening = *((uint16_t *)(table[ _CALIBRATION_SHARP_ALT_UD_FS_HDR]->ptr) +
                table[_CALIBRATION_SHARP_ALT_UD_LINEAR]->rows * table[_CALIBRATION_SHARP_ALT_UD_LINEAR]->cols -1 );
        stab.global_maximum_iridix_strength = *(uint8_t *)(table[_CALIBRATION_IRIDIX_STRENGTH_MAXIMUM_WDR]->ptr);
    }
    stab.global_minimum_temper_strength = *((uint16_t *)(table[ _CALIBRATION_TEMPER_STRENGTH]->ptr) + 1);
    stab.global_maximum_temper_strength = *((uint16_t *)(table[ _CALIBRATION_TEMPER_STRENGTH]->ptr) +
            table[_CALIBRATION_TEMPER_STRENGTH]->rows * table[_CALIBRATION_TEMPER_STRENGTH]->cols -1 );
    stab.global_minimum_iridix_strength = *(uint8_t *)(table[_CALIBRATION_IRIDIX_MIN_MAX_STR]->ptr);
    return 0;
}

int apical_isp_day_or_night_s_ctrl_internal(struct isp_core_dev *isp_core)
{
    unsigned int tmp_top = 0;
    ISP_CORE_MODE_DN_E dn;// TODO
	LookupTable** table = NULL;
	TXispPrivCustomerParamer *customer = NULL;
    TXispPrivParamManage *param;
    int ret;

    tmp_top = APICAL_READ_32(0x40);
    if(dn == ISP_CORE_RUNING_MODE_DAY_MODE) {
        apical_isp_ds1_cs_conv_clip_min_uv_write(0);
        apical_isp_ds1_cs_conv_clip_max_uv_write(1023);
        table = param->isp_param[TX_ISP_PRIV_PARAM_DAY_MODE].calibrations;
        customer = &param->customer[TX_ISP_PRIV_PARAM_DAY_MODE];
    } else {
        apical_isp_ds1_cs_conv_clip_min_uv_write(512);
        apical_isp_ds1_cs_conv_clip_max_uv_write(512);
        table = param->isp_param[TX_ISP_PRIV_PARAM_NIGHT_MODE].calibrations;
        customer = &param->customer[TX_ISP_PRIV_PARAM_NIGHT_MODE];
    }

    if(table && customer) {
        apical_dynamic_calibration(table);
        apical_green_equalization(customer);
        /* Max ISP Digital Gain */
        status = apical_command(TSYSTEM, SYSTEM_MAX_ISP_DIGITAL_GAIN, customer->max_isp_dgain, COMMAND_SET, &ret);
        if(status != 0) {
            log("Custom set max isp digital gain failure!reture value is %d,reason is %d\n",status,ret);
        }
        /* Max Sensor Analog Gain */
        status = apical_command(TSYSTEM, SYSTEM_MAX_SENSOR_ANALOG_GAIN, customer->max_sensor_again, COMMAND_SET, &ret);
        if(status != 0) {
            log("Custom set max isp digital gain failure!reture value is %d,reason is %d\n",status,ret);
        }
        /* modify the node */
        status = apical_command(TIMAGE, WDR_MODE_ID, -1, COMMAND_SET, &ret);
        if(status != 0) {
            log("get wdr mode failure!reture value is %d,reason is %d\n", status, ret);
        }
        set_stab(ret, table);

        tmp_top = (tmp_top | 0x0c02da6c) & (~(customer->top));
        tmp_top |= 0x00fc0000;
        APICAL_WRITE_32(0x40, tmp_top);
        if ((customer->top) & (1 << 27))
            apical_isp_ds1_sharpen_enable_write(1);
        else
            apical_isp_ds1_sharpen_enable_write(0);
    }
    ctrls->daynight = dn;

    return 0;
}

static int isp_core_interrupt_service_routine(struct v4l2_subdev *sd, u32 status, bool *handled)
{
    unsigned short isp_irq_status = 0;
    int ret = IRQ_HANDLED;
    int i;
    struct isp_core_dev *isp_core = (struct isp_core_dev *)v4l2_get_subdevdata(sd);
    unsigned char color = apical_isp_top_rggb_start_read();
    struct apical_isp_contrl *contrl = &isp_core->contrl;

    if((isp_irq_status = apical_isp_interrupts_interrupt_status_read()) != 0) {
        apical_isp_interrupts_interrupt_clear_write(0);
        apical_isp_interrupts_interrupt_clear_write(isp_irq_status);

        for(i = 0; i < APICAL_IRQ_COUNT; i++){
            if(isr_func[i])
                isr_func[i](isr_param[i]);
            if(isp_irq_status & (1 << i)){
                switch(i){
                    case APICAL_IRQ_FRAME_START:
                        log("^~^ frame start ^~^\n");
                        isp_configure_base_addr(isp_core);
                        ret = IRQ_WAKE_THREAD;
                        break;
                    case APICAL_IRQ_FRAME_WRITER_FR:
                        isp_core_update_addr(isp_core);
                        isp_modify_dma_direction(isp_core);
                        break;
                    case APICAL_IRQ_FRAME_WRITER_DS:
                        isp_core_update_addr(isp_core);
                        isp_modify_dma_direction(isp_core);
                        break;
                    case APICAL_IRQ_FRAME_END:
                        if(isp_core->hflip_state == apical_isp_top_bypass_mirror_read())						{
                            if(isp_core->hflip_state){
                                color ^= 1;
                            }else{
                                if(contrl->pattern != color){
                                    color ^= 1;
                                }
                            }
                            apical_isp_top_rggb_start_write(color);
                            apical_isp_top_bypass_mirror_write(isp_core->hflip_state ?0:1);
                        }
                        /* APICAL_WRITE_32(0x18,2);  */
                        isp_core->frame_state = 0;
                        isp_configure_base_addr(isp_core);
                        isp_modify_dma_direction(isp_core);
                        if(isp_core->dma_state != 1){
                            isp_enable_channel(isp_core);
                        }
                        if (1 == isp_core->isp_daynight_switch) {
                            int ret = 0;
                            ret = apical_isp_day_or_night_s_ctrl_internal(isp_core);
                            if (ret)
                                log("%s[%d] apical_isp_day_or_night_s_ctrl_internal failed!\n", __func__, __LINE__);
                            isp_core->isp_daynight_switch = 0;
                        }
                    case APICAL_IRQ_AE_STATS:
                    case APICAL_IRQ_AWB_STATS:
                    case APICAL_IRQ_AF_STATS:
                    case APICAL_IRQ_FPGA_FRAME_START:
                    case APICAL_IRQ_FPGA_FRAME_END:
                    case APICAL_IRQ_FPGA_WDR_BUF:
                    case APICAL_IRQ_DS1_OUTPUT_END:
                        isp_modify_dma_direction(isp_core);
                        if(isp_core->dma_state != 1){
                            isp_enable_channel(isp_core);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }

    return 0;
}

static const struct v4l2_subdev_core_ops isp_core_subdev_core_ops = {
	.init = isp_core_ops_init,
    .interrupt_service_routine = isp_core_interrupt_service_routine,
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

int vic_video_s_stream(struct v4l2_subdev *sd, int enable)
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
    .s_stream = vic_video_s_stream,
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

static void isp_enable_irq(isp_device_t *ispdev, int enable)
{
	volatile unsigned int reg = isp_readl(ispdev->irqbase, ISP_TOP_IRQ_ENABLE);
	unsigned long flags;

	reg |= enable;
	isp_writel(ispdev->irqbase, ISP_TOP_IRQ_ENABLE, reg);
	spin_lock_irqsave(&ispdev->slock, flags);
	if(ispdev->state == 0){
		ispdev->state = 1;
		enable_irq(ispdev->irq);
	}
	spin_unlock_irqrestore(&ispdev->slock, flags);
}

static void isp_disable_irq(isp_device_t *ispdev, int enable)
{
	volatile unsigned int reg = isp_readl(ispdev->irqbase, ISP_TOP_IRQ_ENABLE);
	unsigned long flags;

	spin_lock_irqsave(&ispdev->slock, flags);
	reg &= ~enable;
	isp_writel(ispdev->irqbase, ISP_TOP_IRQ_ENABLE, reg);
	if(reg == 0 && ispdev->state){
		ispdev->state = 0;
		disable_irq(ispdev->irq);
	}
	spin_unlock_irqrestore(&ispdev->slock, flags);
}

static void isp_unmask_irq(isp_device_t *ispdev, int mask)
{
	volatile unsigned int reg = isp_readl(ispdev->irqbase, ISP_TOP_IRQ_MASK);

	reg &= ~mask;
	isp_writel(ispdev->irqbase, ISP_TOP_IRQ_MASK, reg);
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
	isp_disable_irq(ispdev, ISP_TOP_IRQ_ALL);
	isp_unmask_irq(ispdev, ISP_TOP_IRQ_ISP);
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
    ispdevp = ispdev;
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

void system_set_interrupt_handler(uint8_t source, system_interrupt_handler_t handler, void* param)
{
    log("set interrupt handler, called by apical");
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
    log("clear irq souces, called by apical");
	isp_clear_irq_source();
}

void system_hw_interrupts_disable(void)
{
    log("disable irq ISP_TOP_IRQ_ISP, called by apical");
    isp_disable_irq(ispdevp, ISP_TOP_IRQ_ISP);
}

void system_hw_interrupts_enable(void)
{
    log("enable irq ISP_TOP_IRQ_ISP, called by apical");
    isp_enable_irq(ispdevp, ISP_TOP_IRQ_ISP);
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

