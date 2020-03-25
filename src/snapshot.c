#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

struct frame_image_scalercap {
	unsigned short max_width;
	unsigned short max_height;
	unsigned short min_width;
	unsigned short min_height;
};
struct frame_image_scaler {
	unsigned short out_width;
	unsigned short out_height;
};

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define SENSOR_NAME "jxh62"
#define VIDIOC_REGISTER_SENSOR _IOC(_IOC_WRITE, 0x56, 0xc1, 0x50)
#define BASE_VIDIOC_PRIVATE	192		/* 192-255 are private */
#define VIDIOC_DEFAULT_CMD_ISP_TUNING _IOC(_IOC_READ|_IOC_WRITE, 0x56, 0xc6, 0xc)
#define VIDIOC_DEFAULT_CMD_SCALER_CAP	 _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct frame_image_scalercap)
#define VIDIOC_DEFAULT_CMD_SET_SCALER	 _IOW('V', BASE_VIDIOC_PRIVATE + 4, struct frame_image_scaler)
#define IMAGE_TUNING_CID_CONTROL_FPS  (0x08000000+0x20*7)
#define log(fmt, args...) do { \
    char *s = strrchr(__FILE__, '/'); \
    printf("%s:%d(%s) $ "fmt"\n", s+1, __LINE__, __FUNCTION__, ##args);\
} while(0)

enum io_method {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
};

struct buffer {
	void   *start;
	size_t  length;
};

enum tx_isp_priv_ioctl_direction {
	TX_ISP_PRIVATE_IOCTL_SET,
	TX_ISP_PRIVATE_IOCTL_GET,
};

struct isp_image_tuning_default_ctrl {
	enum tx_isp_priv_ioctl_direction dir;
	struct v4l2_control control;
};

typedef enum {
	TX_SENSOR_CONTROL_INTERFACE_I2C = 1,	/**< I2C控制总线 */
	TX_SENSOR_CONTROL_INTERFACE_SPI,	/**< SPI控制总线 */
} IMPSensorControlBusType;

typedef struct {
	char type[20];		/**< I2C设备名字，必须与摄像头驱动中struct i2c_device_id中name变量一致 */
	int addr;		/**< I2C地址 */
	int i2c_adapter_id;	/**< I2C控制器 */
} IMPI2CInfo;
/**
* 摄像头控制总线类型是SPI时，需要配置的参数结构体
*/
typedef struct {
	char modalias[32];	/**< SPI设备名字，必须与摄像头驱动中struct spi_device_id中name变量一致 */
	int bus_num;		/**< SPI总线地址 */
} IMPSPIInfo;

typedef struct {
	char name[32];					/**< 摄像头名字 */
	IMPSensorControlBusType cbus_type;	/**< 摄像头控制总线类型 */
	union {
		IMPI2CInfo i2c;				/**< I2C总线信息 */
		IMPSPIInfo spi;				/**< SPI总线信息 */
	};
	unsigned short rst_gpio;		/**< 摄像头reset接口链接的GPIO，注意：现在没有启用该参数 */
	unsigned short pwdn_gpio;		/**< 摄像头power down接口链接的GPIO，注意：现在没有启用该参数 */
	unsigned short power_gpio;		/**< 摄像头power 接口链接的GPIO，注意：现在没有启用该参数 */
} IMPSensorInfo;

static int              fd_subdev = -1;
static int              fd_video0 = -1;
static int              fd_video1 = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;

static void errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int open_device(char *dev_name)
{
	struct stat st;
    int fd = -1;

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no devicen", dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

    return fd;
}

static void process_image(const void *p, int size)
{
    log("got frame addr:%p size:%d", p, size);
    FILE *fp = fopen("./capture.jpg", "w");

    fwrite(p, size, 1, fp);
    fclose(fp);
}

static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd_video1, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
            case EAGAIN:
                return 0;
            case EIO:
                /* Could ignore EIO, see spec. */
                /* fall through */
            default:
                errno_exit("VIDIOC_DQBUF");
        }
    }

    for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long)buffers[i].start
                && buf.length == buffers[i].length)
            break;

    assert(i < n_buffers);
    process_image((void *)buf.m.userptr, buf.bytesused);
    if (-1 == xioctl(fd_video1, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    return 1;
}

static void mainloop(void)
{
	unsigned int count = 1;

	while (count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd_video1, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select(fd_video1 + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
				fprintf(stderr, "select timeout\\n");
				exit(EXIT_FAILURE);
			}

			if (read_frame())
				break;
			/* EAGAIN - continue select loop. */
		}
	}
}

static void init_device(void)
{
    struct v4l2_format fmt;
    unsigned int min;
    IMPSensorInfo sensor_info;
    int ret = 0;
    struct v4l2_input input;

    fd_subdev = open_device("/dev/v4l-subdev0");

    /*1.注册sensor jxh62*/
    memset(&sensor_info, 0, sizeof(IMPSensorInfo));
    memcpy(sensor_info.name, SENSOR_NAME, sizeof(SENSOR_NAME));
    sensor_info.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    memcpy(sensor_info.i2c.type, SENSOR_NAME, sizeof(SENSOR_NAME));
    sensor_info.i2c.addr = 0x30;
    if (-1 == xioctl(fd_subdev, VIDIOC_REGISTER_SENSOR, &sensor_info)) {
        log("VIDIOC_REGISTER_SENSOR %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("register sensor:%s success", SENSOR_NAME);
    /*2.枚举输入源*/
    input.index = 0;
    do {
        ret = xioctl(fd_subdev, VIDIOC_ENUMINPUT, &input);
        input.index++;
        if (!ret) {
            log("name:%s type:%d status:%d capabilities:%d", input.name, input.type, input.status, input.capabilities);
        }
    } while(ret != -1 && errno != EINVAL);

    /*3. 选择输入源*/
    int index = 0;
    if (-1 == xioctl(fd_subdev, VIDIOC_S_INPUT, &index)) {
        log("VIDIOC_S_INPUT %s", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /*4. 获取当前输入源*/
    if (-1 == xioctl(fd_subdev, VIDIOC_G_INPUT, &index)) {
        log("VIDIOCS_G_INPUT %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("current ipupt:%d", index);
    /*5. 打开流*/
    int arg = 0;
    if (-1 == xioctl(fd_subdev, VIDIOC_STREAMON, &arg)) {
        log("VIDIOC_STREAMON %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("v4l-subdev0 stream on success");

    fd_video0 = open("/dev/video0", O_RDWR|O_CLOEXEC);
    log("fd_video0:%d", fd_video0);
    /*6. 设置帧率:0x19 = 25fps*/
    log("+++ try to set fps");
    struct isp_image_tuning_default_ctrl fps_info;
    int fps = 0x190001;
    fps_info.dir = TX_ISP_PRIVATE_IOCTL_SET;
    fps_info.control.id = IMAGE_TUNING_CID_CONTROL_FPS;
    fps_info.control.value = &fps;
    if (-1 == xioctl(fd_video0, VIDIOC_DEFAULT_CMD_ISP_TUNING, &fps_info)) {
        log("IMAGE_TUNING_CID_CONTROL_FPS %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    /*7. 获取亮度*/
    struct v4l2_control control;
    control.id = V4L2_CID_BRIGHTNESS;
    if (-1 == xioctl(fd_video0, VIDIOC_G_CTRL, &control)) {
        log("V4L2_CID_BRIGHTNESS %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("brightness:%d", control.value);
    /*8. 获取饱和度*/
    control.id = V4L2_CID_SATURATION;
    if (-1 == xioctl(fd_video0, VIDIOC_G_CTRL, &control)) {
        log("V4L2_CID_SATURATION %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("saturation:%d", control.value);
    /*9. 获取关于视频裁剪和缩放的能力: 分辨率*/
    fd_video1 = open_device("/dev/video1");
    struct v4l2_cropcap cap;
    if (-1 == xioctl(fd_video1, VIDIOC_CROPCAP, &cap)) {
        log("VIDIOC_CROPCAP %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("crop cap = type: %d bounds {x:%d y:%d w:%d h:%d} defrect {x:%d y:%d w:%d h:%d}", cap.type,
            cap.bounds.left, cap.bounds.top, cap.bounds.width, cap.bounds.height,
            cap.defrect.left, cap.defrect.top, cap.defrect.width, cap.defrect.height);
    /*10. 设置裁剪后的矩形*/
    struct v4l2_crop crop;
    crop.type = 1;
    crop.c.left = 0;
    crop.c.top = 0;
    crop.c.width = cap.bounds.width;
    crop.c.height = cap.bounds.height;
    if (-1 == xioctl(fd_video1, VIDIOC_S_CROP, &crop)) {
        log("VIDIOC_S_CROP %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    /*11. 获取裁剪后的矩形*/
    if (-1 == xioctl(fd_video1, VIDIOC_G_CROP, &crop)) {
        log("VIDIOC_S_CROP %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("current crop : type: %d bounds {x:%d y:%d w:%d h:%d}", 
            crop.type, crop.c.left, crop.c.top, crop.c.width, crop.c.height);
    /*
     * 12. Scaler的实现最简单的就是插行/抽行，就是利用行内相邻像素或者
     * 行间的相邻行在原始图像里直接插入或删除实现up或者down。复杂
     * 点的就是利用行内多像素点实现，对不同像素点进行加权运算得出
     * 新的像素点，垂直方向利用行间也同样的加权。
     * 对不同分辨率进行调整，以适应显示屏
     * */
    struct frame_image_scalercap scalercap;
    if (-1 == xioctl(fd_video1, VIDIOC_DEFAULT_CMD_SCALER_CAP, &scalercap)) {
        log("VIDIOC_DEFAULT_CMD_SCALER_CAP %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("scaler{max_w:%d max_h:%d min_w:%d min_h:%d}", 
            scalercap.max_width, scalercap.max_height, scalercap.min_width, scalercap.min_height);
    /*13. 设置分辨率调整*/
    struct frame_image_scaler scaler;
    scaler.out_width = scalercap.max_width;
    scaler.out_height = scalercap.max_height;
    if (-1 == xioctl(fd_video1, VIDIOC_DEFAULT_CMD_SET_SCALER, &scaler)) {
        log("VIDIOC_DEFAULT_CMD_SET_SCALER %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    /*14. 尝试设置某种视频格式*/
    struct v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = 1280;
    format.fmt.pix.height = 720;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR12;//V4L2_PIX_FMT_NV12;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    format.fmt.pix.bytesperline = 0;
    format.fmt.pix.sizeimage = 0;
    format.fmt.pix.colorspace = 0;
    if (-1 == xioctl(fd_video1, VIDIOC_TRY_FMT, &format)) {
        log("VIDIOC_TRY_FMT %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    // try了某种format之后，如果不支持，底层驱动会改写v4l2_format结构里面的相应的成员。
    // 比如这里field传了V4L2_FIELD_ANY进去，底层驱动会把这个改写成V4L2_FIELD_INTERLACED
    log("fileld after try:%d", format.fmt.pix.field);
    /*15. 设置视频格式*/
    if (-1 == xioctl(fd_video1, VIDIOC_S_FMT, &format)) {
        log("VIDIOC_S_FMT %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    log("sizeimage:%d", format.fmt.pix.sizeimage);
    /*16. 三种方式读取一帧数据
     *  a. 调用read
     *  b. 内核申请内存，告诉用户层,用户层map到自己的空间
     *  c. 用户层申请内存，告诉内核层，内核层使用
     *  我们使用用户层申请内存
     *  */
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctl(fd_video1, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            log("/dev/video0 does not support user pointer i/on");
            exit(EXIT_FAILURE);
        } else {
            log("VIDIOC_REQBUFS %s", strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    log("req.count:%d", req.count);
    buffers = calloc(4, sizeof(*buffers));

    if (!buffers) {
        fprintf(stderr, "Out of memory\\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = format.fmt.pix.sizeimage;
        buffers[n_buffers].start = malloc(format.fmt.pix.sizeimage);

        if (!buffers[n_buffers].start) {
            fprintf(stderr, "Out of memory\\n");
            exit(EXIT_FAILURE);
        }
    }

    /*17. 用户申请的内存地址告诉内核*/
    int i = 0;
    for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers[i].start;
        buf.length = buffers[i].length;

        if (-1 == xioctl(fd_video1, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
    }
    /*18. 启动视频流*/
	enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd_video1, VIDIOC_STREAMON, &type))
        log("VIDIOC_STREAMON %s", strerror(errno));
}

static void close_device(void)
{
	if (-1 == close(fd_subdev))
		errno_exit("close");
	if (-1 == close(fd_video0))
		errno_exit("close");
	if (-1 == close(fd_video1))
		errno_exit("close");

}

int main(int argc, char **argv)
{
	init_device();
    mainloop();
	close_device();

	return 0;
}
