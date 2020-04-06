/**
* @file src/capture.c
* @author rigensen
* @brief 
* @date 四  3/26 22:00:09 2020
*/

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

#define log(fmt, args...) do { \
    char *s = strrchr(__FILE__, '/'); \
    printf("%s:%d(%s) $ "fmt"\n", s+1, __LINE__, __FUNCTION__, ##args);\
} while(0)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct buffer {
    void   *start;
    size_t  length;
};
struct buffer          *buffers;
static unsigned int     n_buffers;

static void init_userp(int fd, unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req)) {
        log("VIDIOC_REQBUFS error(%s)", strerror(errno));
        return;
    }
    buffers = calloc(4, sizeof(*buffers));
    if (!buffers) {
        log("Out of memory");
        return;
    }
    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = malloc(buffer_size);

        if (!buffers[n_buffers].start) {
            log("Out of memory");
            return;
        }
    }
}

static void process_image(const void *p, int size)
{
    log("size:%d", size);
}

static int read_frame(int fd)
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;

    if (-1 == ioctl(fd, VIDIOC_DQBUF, &buf)) {
        log("VIDIOC_DQBUF error(%s)", strerror(errno));
        return 0;
    }

    for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long)buffers[i].start
                && buf.length == buffers[i].length)
            break;

    assert(i < n_buffers);
    process_image((void *)buf.m.userptr, buf.bytesused);
    if (-1 == ioctl(fd, VIDIOC_QBUF, &buf)) {
        log("VIDIOC_QBUF error(%s)", strerror(errno));
        return 0;
    }

    return 0;
}

static void mainloop(int fd)
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (-1 == r) {
            if (EINTR == errno)
                continue;
            log("select");
        }

        if (0 == r) {
            log("select timeout");
            return;
        }

        if (read_frame(fd))
            break;
        /* EAGAIN - continue select loop. */
    }
}

int main()
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    int min = 0, i = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int fd = open("/dev/video0", O_RDWR | O_NONBLOCK, 0);

    if(fd <0) {
        log("open error(%s)", strerror(errno));
        return 0;
    }
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        log("VIDIOC_QUERYCAP error(%s)", strerror(errno));
        return 0;
    }
    log("driver:%s card:%s buf_info:%s version:%d", cap.driver, cap.card, cap.bus_info, cap.version);
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        log("/dev/video0 is not video capture device");
        return 0;
    }
    if (ioctl(fd, VIDIOC_CROPCAP, &cropcap) < 0) {
        log("VIDIOC_CROPCAP error(%s)", strerror(errno));
        return 0;
    }
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */
    if (ioctl(fd, VIDIOC_S_CROP, &crop) < 0) {
        log("VIDIOC_CROPCAP error(%s)", strerror(errno));
        return 0;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        log("VIDIOC_G_FMT error(%s)", strerror(errno));
        return 0;
    }
    log("type : %d fmt.pix {w:%d h:%d pixelformat:%d field:%d bytesperline:%d sizeimage:%d colorspace:%d}",
            fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat, fmt.fmt.pix.field, fmt.fmt.pix.bytesperline,
            fmt.fmt.pix.sizeimage, fmt.fmt.pix.colorspace);
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    log("sizeimage:%d", fmt.fmt.pix.sizeimage);
    init_userp(fd, fmt.fmt.pix.sizeimage);
    for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers[i].start;
        buf.length = buffers[i].length;

        if (-1 == ioctl(fd, VIDIOC_QBUF, &buf)) {
            log("VIDIOC_QBUF err(%s)", strerror(errno));
            return 0;
        }
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(fd, VIDIOC_STREAMON, &type)) {
        log("VIDIOC_STREAMON err(%s)", strerror(errno));
    }
    mainloop(fd);
    if (-1 == ioctl(fd, VIDIOC_STREAMOFF, &type))
        log("VIDIOC_STREAMOFF error(%s)", strerror(errno));
    for (i = 0; i < n_buffers; ++i)
        free(buffers[i].start);
    close(fd);
    return 0;
}

