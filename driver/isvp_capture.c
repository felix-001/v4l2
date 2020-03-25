/**
 * @file isvp_capture.c
 * @author rigensen
 * @brief 
 * @date 三  3/25 21:35:00 2020
 */
#include <linux/module.h>
#include <linux/platform_device.h>

#define log(fmt, args...) printk("%s:%d $ "fmt"\n", __func__, __LINE__, ##args)

static int isp_probe(struct platform_device *pdev)
{
    log("isp probe");
    return 0;
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

