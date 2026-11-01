/* Synthetic, disposable-child-only probe. Never preload in a real ROS stack.
 * Replaces rcl_publish with a successful no-op: NOTHING is delivered to DDS.
 * Pointer-only ABI and int rcl_ret_t match this Linux ROS installation.
 */
#define _DEFAULT_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int rcl_publish(const void *publisher, const void *message, void *allocation)
{
    (void)publisher; (void)message; (void)allocation;
    int (*gil_check)(void) = dlsym(RTLD_DEFAULT, "PyGILState_Check");
    const char *delay = getenv("IMU_RESEARCH_DELAY_US");
    fprintf(stderr, "[synthetic-rcl-publish] gil_held=%d delay_us=%s\n",
            gil_check ? gil_check() : -1, delay ? delay : "0");
    if (delay) usleep((useconds_t)strtoul(delay, NULL, 10));
    return 0;
}
