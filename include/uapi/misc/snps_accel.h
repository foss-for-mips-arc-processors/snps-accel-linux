/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (C) 2023 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef __SNPS_ACCEL_H__
#define __SNPS_ACCEL_H__

#include "linux/types.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define SNPS_ACCEL_MAGIC		'N'

#define SNPS_ACCEL_INFO_SHMEM		0x01
#define SNPS_ACCEL_INFO_NOTIFY		0x02
#define SNPS_ACCEL_WAIT_IRQ		0x03
#define SNPS_ACCEL_DMABUF_ALLOC		0x04
#define SNPS_ACCEL_DMABUF_INFO		0x05
#define SNPS_ACCEL_DMABUF_IMPORT	0x06
#define SNPS_ACCEL_DMABUF_DETACH	0x07
#define SNPS_ACCEL_INFO_CLUSTER		0x08
#define SNPS_ACCEL_DMABUF_SYNC_RANGE	0x09

#define SNPS_ACCEL_IOCTL_INFO_SHMEM	\
	_IOR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_INFO_SHMEM, struct snps_accel_shmem)
#define SNPS_ACCEL_IOCTL_INFO_NOTIFY	\
	_IOR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_INFO_NOTIFY, struct snps_accel_notify)
#define SNPS_ACCEL_IOCTL_WAIT_IRQ	\
	_IOWR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_WAIT_IRQ, struct snps_accel_wait_irq)
#define SNPS_ACCEL_IOCTL_DMABUF_ALLOC	\
	_IOWR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_DMABUF_ALLOC, struct snps_accel_dmabuf_alloc)
#define SNPS_ACCEL_IOCTL_DMABUF_INFO	\
	_IOWR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_DMABUF_INFO, struct snps_accel_dmabuf_info)
#define SNPS_ACCEL_IOCTL_DMABUF_IMPORT	\
	_IOW(SNPS_ACCEL_MAGIC, SNPS_ACCEL_DMABUF_IMPORT, struct snps_accel_dmabuf_import)
#define SNPS_ACCEL_IOCTL_DMABUF_DETACH	\
	_IOW(SNPS_ACCEL_MAGIC, SNPS_ACCEL_DMABUF_DETACH, struct snps_accel_dmabuf_detach)
#define SNPS_ACCEL_IOCTL_INFO_CLUSTER	\
	_IOR(SNPS_ACCEL_MAGIC, SNPS_ACCEL_INFO_CLUSTER, struct snps_accel_cluster_info)
#define SNPS_ACCEL_IOCTL_DMABUF_SYNC_RANGE	\
	_IOW(SNPS_ACCEL_MAGIC, SNPS_ACCEL_DMABUF_SYNC_RANGE, struct snps_accel_sync_range)

struct snps_accel_shmem {
	/* Shared memory intermediate offset for use in mmap */
	__u64 offset;

	/* Size of mapped region */
	__u64 size;
};

struct snps_accel_notify {
	/* Shared memory intermediate offset for use in mmap */
	__u64 offset;

	/* Size of mapped region */
	__u64 size;
};

#define SNPS_ACCEL_CLUSTER_FLAG_VIRT	(1u << 0)

struct snps_accel_cluster_info {
	/* ARCSync mode flags */
	__u32 flags;

	/* Host cluster/VP ID */
	__u32 host_cluster_id;

	/* NPX/VPX target cluster/VP ID */
	__u32 target_cluster_id;
};

struct snps_accel_wait_irq {
	/* Timeout in milliseconds for blocking wait operation */
	__u32 timeout;
	/* Total interrupt count returned by the driver */
	__u32 count;
};

enum {
	SNPS_ACCEL_IO_R = 0x1,
	SNPS_ACCEL_IO_W = 0x2
};

struct snps_accel_dmabuf_alloc {
	/* dma-buf file descriptor */
	__s32 fd;

	/* Flags to allpy for dma buffer device mappings */
	__u32 flags;

	/* Size of dma buffer to allocate */
	__u64 size;
};

struct snps_accel_dmabuf_info {
	/* dma-buf file descriptor */
	__s32 fd;

	/* Address as it seen by DMA of a device */
	__u64 addr;

	/* Size of dma buffer to allocate */
	__u64 size;
};

struct snps_accel_dmabuf_import {
	/* dma-buf file descriptor of extermal buffer */
	__s32 fd;
};

struct snps_accel_dmabuf_detach {
	/* dma-buf file descriptor */
	__s32 fd;
};

/*
 * Cache maintenance of a byte range of a dma-buf exported by this driver, for
 * a CPU mapping that is cacheable while the device does not snoop CPU caches.
 * Issued on the device fd; the buffer is named by its dma-buf fd, and any
 * open device fd of this driver works.
 *
 * TO_DEVICE: the CPU wrote the range and the device will read it; writes
 * dirty cache lines back (clean). Also issue it before the device writes a
 * range the CPU has written to, so no dirty line is written back over the
 * device's data later.
 * TO_CPU: the device wrote the range and the CPU will read it; drops the
 * range's cache lines (invalidate), after the device has finished.
 * Both: clean, then invalidate.
 *
 * The maintenance is by cache line: a line partly inside the range is
 * cleaned and invalidated as a whole, so bytes of it outside the range must
 * not be written by the CPU and the device at the same time.
 *
 * A zero length is valid and does nothing. A driver without the call returns
 * -ENOTTY; user space can probe for it with flags = 0, which this driver
 * rejects with -EINVAL.
 */
#define SNPS_ACCEL_SYNC_TO_DEVICE	(1u << 0)
#define SNPS_ACCEL_SYNC_TO_CPU		(1u << 1)

struct snps_accel_sync_range {
	/* dma-buf file descriptor of a buffer exported by this driver */
	__s32 fd;

	/* SNPS_ACCEL_SYNC_TO_DEVICE, SNPS_ACCEL_SYNC_TO_CPU or both */
	__u32 flags;

	/* Byte range inside the buffer */
	__u64 offset;
	__u64 length;
};

#if defined(__cplusplus)
}
#endif

#endif  /* __SNPS_ACCEL_H__ */
