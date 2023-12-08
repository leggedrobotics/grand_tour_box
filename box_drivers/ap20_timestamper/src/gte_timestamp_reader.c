/*
 * tegra_gte_mon - monitor GPIO line events from userspace and hardware
 * timestamp.
 *
 * Copyright (C) 2020 Dipen Patel
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Example Usage:
 *	tegra_gte_mon -d <device> -g <global gpio pin> -r -f
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
//#include <linux/tegra-gte-ioctl.h>
#include "tegra-gte-ioctl.h"

#include <sys/types.h>
#include <sys/stat.h>

int monitor_device(const char *device_name,
		   unsigned int gnum,
		   unsigned int eventflags,
		   unsigned int loops,
		   int fifo_file)
{
	struct tegra_gte_hts_event_req req = {0};
	struct tegra_gte_hts_event_data event;
	int fd;
	int ret;
	uint64_t i = 0;

	printf("before open");
	fd = open(device_name, 0);
	if (fd == -1) {
		ret = -errno;
		perror("Error: ");
		goto exit_close_error;
	}
	printf("after open");

	req.global_gpio_pin = gnum;
	req.eventflags = eventflags;

	printf("ioctl");
	ret = ioctl(fd, TEGRA_GTE_HTS_CREATE_GPIO_EV_IOCTL, &req);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET EVENT "
			"IOCTL (%d)\n",
			ret);
		goto exit_close_error;
	}

	fprintf(stdout, "Monitoring line %d on %s\n", gnum, device_name);

	while (1) {
		ret = read(req.fd, &event, sizeof(event));
		if (ret == -1) {
			if (errno == -EAGAIN) {
				fprintf(stderr, "nothing available\n");
				continue;
			} else {
				ret = -errno;
				fprintf(stderr, "Failed to read event (%d)\n",
					ret);
				break;
			}
		}

		if (ret != sizeof(event)) {
			fprintf(stderr, "Reading event failed\n");
			ret = -EIO;
			break;
		}

		fprintf(stdout, "HW timestamp GPIO EVENT %" PRIu64 "\n",
			//event.timestamp);
			i);

		char timestamp_str[21];
		sprintf(timestamp_str, "%" PRIu64, i);

		write(fifo_file, timestamp_str, sizeof(event.timestamp));
	}
	i++;

exit_close_error:
	if (close(fd) == -1)
		perror("Failed to close GPIO character device file");
	return ret;
}


int main(int argc, char **argv)
{
	const char device_name[] = "/dev/gtechip0";
	unsigned int gnum = 325;
	unsigned int loops = 0;
	unsigned int eventflags = TEGRA_GTE_EVENT_RISING_EDGE;
	printf("start");
	const char *fifoPath = "/tmp/fifo1";
	int mkfifo_err = mkfifo(fifoPath, 0666);
	if (mkfifo_err == -1) {
		perror("Error mkfifo");
	}

	printf("here");

	int fifo_file = open(fifoPath, O_WRONLY);
	if (fifo_file == -1) {
        perror("Error opening FIFO for writing");
        return 1;
    }
	printf("before");
	return monitor_device(device_name, gnum, eventflags, loops, fifo_file);
}
