/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2024 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _SNPS_ACCEL_MEM_H
#define _SNPS_ACCEL_MEM_H

#include <linux/device.h>
#include <linux/dma-buf.h>

struct snps_accel_app_mm;
struct snps_accel_sync_range;

#define SNPS_ACCEL_MAX_MEM_REGIONS 16

/**
 * struct snps_accel_mem_region - reserved memory region descriptor
 * @dev:  child device bound to the region's reserved memory
 * @base: physical base address of the region (0 if unknown)
 * @size: size of the region in bytes (0 if unknown)
 */
struct snps_accel_mem_region {
	struct device *dev;
	phys_addr_t base;
	phys_addr_t size;
};

/**
 * struct snps_accel_dmabuf_attachment - buffer attachment description
 */
struct snps_accel_dmabuf_attachment {
	struct device *dev;
	struct sg_table sgt;
	bool mapped;
	struct list_head node;
};

/**
 * struct snps_accel_mem_buffer - memory buffer description structure
 */
struct snps_accel_mem_buffer {
	struct snps_accel_mem_ctx *ctx;
	struct list_head ctx_link;
	struct device *dev;
	struct dma_buf *dmabuf;
	enum dma_data_direction dma_dir;
	bool imported;
	dma_addr_t da;
	void *va;
	phys_addr_t pa;
	size_t size;
	bool mapped;
	struct sg_table *dmasgt;
	struct dma_buf_attachment *import_attach;
	struct mutex lock;
	struct list_head attachments;
};

/**
 * struct snps_accel_mem_ctx - the driver cilent memory context description
 */
struct snps_accel_mem_ctx {
	struct device *dev;
	struct mutex list_lock;
	struct list_head mlist;
	u32 num_regions;
	struct snps_accel_mem_region regions[SNPS_ACCEL_MAX_MEM_REGIONS];
};

void snps_accel_app_mem_init(struct device *dev, struct snps_accel_mem_ctx *mem);
void snps_accel_app_mem_init_regions(struct snps_accel_mem_ctx *mem,
				     struct snps_accel_mem_region *regions,
				     u32 num_regions);
void snps_accel_app_release_import(struct snps_accel_mem_ctx *mem);
struct snps_accel_mem_buffer *snps_accel_app_dmabuf_create(struct snps_accel_mem_ctx *mem,
							   u64 size, u32 dflags);
void snps_accel_app_dmabuf_release(struct snps_accel_mem_buffer *mbuf);
int snps_accel_app_dmabuf_info(struct snps_accel_mem_ctx *mem, struct snps_accel_dmabuf_info *info);
int snps_accel_app_dmabuf_import(struct snps_accel_mem_ctx *mem, int fd);
int snps_accel_app_dmabuf_detach(struct snps_accel_mem_ctx *mem, int fd);
int snps_accel_app_dmabuf_sync_range(const struct snps_accel_sync_range *r);

#endif  /* _SNPS_ACCEL_MEM_H */
