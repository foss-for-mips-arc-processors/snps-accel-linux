// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2025 Synopsys, Inc. (www.synopsys.com)
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/overflow.h>
#include <linux/slab.h>

#include <uapi/misc/snps_accel.h>
#include "snps_accel_drv.h"

static bool snps_accel_page_in_region(const struct snps_accel_mem_region *region,
				      struct page *page, size_t size)
{
	phys_addr_t pa = page_to_phys(page);

	if (!region->size)
		return true;

	return pa >= region->base &&
	       pa + size <= region->base + region->size;
}

static struct snps_accel_mem_buffer *
snps_accel_mbuf_alloc(struct snps_accel_mem_ctx *mem, size_t size,
		      enum dma_data_direction dma_dir)
{
	struct page *page;
	struct snps_accel_mem_buffer *mbuf = NULL;
	struct snps_accel_file_priv *fpriv = to_snps_accel_file_priv(mem);
	struct device *dmabuf_dev;
	size_t aligned_size = PAGE_ALIGN(size);

	mbuf = kzalloc(sizeof(*mbuf), GFP_KERNEL);
	if (!mbuf)
		return NULL;

	if (mem->num_regions <= 1) {
		dmabuf_dev = mem->dev;

		page = dma_alloc_pages(dmabuf_dev, aligned_size, &mbuf->da,
				       dma_dir, GFP_KERNEL | __GFP_NOWARN);
		if (!page) {
			dev_err(mem->dev, "Failed to allocate contiguous memory for buffer\n");
			kfree(mbuf);
			return NULL;
		}
	} else {
		/* Try each region in order until allocation succeeds */
		int i;

		page = NULL;

		for (i = 0; i < mem->num_regions && !page; i++) {
			dmabuf_dev = mem->regions[i].dev;

			page = dma_alloc_pages(dmabuf_dev, aligned_size, &mbuf->da,
					       dma_dir, GFP_KERNEL | __GFP_NOWARN);

			/*
			 * Check that an allocated block lies within the reserved region
			 * it was requested from. On CMA (shared-dma-pool) exhaustion
			 * dma_alloc_pages() silently falls back to the buddy allocator.
			 * Detect this condition and move on to the next region instead.
			 */
			if (page &&
			    !snps_accel_page_in_region(&mem->regions[i], page,
						       aligned_size)) {
				dev_dbg(mem->dev,
					"Region %u fell back to non-reserved memory\n",
					i);
				dma_free_pages(dmabuf_dev, aligned_size, page,
					       mbuf->da, dma_dir);
				page = NULL;
			}

			if (page) {
				if (i > 0)
					dev_dbg(mem->dev,
						"Allocated %zu bytes from region %d (fallback)\n",
						aligned_size, i);
			} else if (i < mem->num_regions - 1) {
				dev_dbg(mem->dev,
					"Region %d failed for %zu bytes, trying next\n",
					i, aligned_size);
			}
		}

		if (!page) {
			dev_err(mem->dev,
				"Failed to allocate %zu bytes after trying %d regions\n",
				aligned_size, mem->num_regions);
			kfree(mbuf);
			return NULL;
		}
	}

	mbuf->ctx = mem;
	mbuf->dev = dmabuf_dev;
	mbuf->va = page_address(page);
	mbuf->pa =  page_to_pfn(page) << PAGE_SHIFT;
	mbuf->size = aligned_size;
	mbuf->dma_dir = dma_dir;

	/* Flush stale CPU cache lines before the device writes to the buffer */
	dma_sync_single_for_device(mbuf->dev, mbuf->da, mbuf->size, dma_dir);

	mutex_init(&mbuf->lock);
	INIT_LIST_HEAD(&mbuf->attachments);

	mutex_lock(&mem->list_lock);
	list_add(&mbuf->ctx_link, &mem->mlist);
	mutex_unlock(&mem->list_lock);

	snps_accel_file_priv_get(fpriv);
	return mbuf;
}

static void
snps_accel_mbuf_free(struct snps_accel_mem_ctx *mem,
		     struct snps_accel_mem_buffer *mbuf,
		     enum dma_data_direction dma_dir)
{
	struct snps_accel_file_priv *fpriv = to_snps_accel_file_priv(mem);

	mutex_lock(&mem->list_lock);
	list_del(&mbuf->ctx_link);
	mutex_unlock(&mem->list_lock);

	dma_free_pages(mbuf->dev, mbuf->size,
		       virt_to_page(mbuf->va),
		       mbuf->da, dma_dir);

	kfree(mbuf);
	snps_accel_file_priv_put(fpriv);
}

static struct snps_accel_mem_buffer *
snps_accel_dmabuf_find_by_dmabuf_locked(struct snps_accel_mem_ctx *mem,
					struct dma_buf *dmabuf)
{
	struct snps_accel_mem_buffer *mbuf;

	/* Caller must hold mem->list_lock */
	list_for_each_entry(mbuf, &mem->mlist, ctx_link) {
		if (mbuf->dmabuf == dmabuf)
			return mbuf;
	}

	return NULL;
}

static bool snps_accel_dmabuf_is_contig(struct sg_table *sgt)
{
	struct scatterlist *s;
	dma_addr_t expected = sg_dma_address(sgt->sgl);
	unsigned int i;

	for_each_sgtable_dma_sg(sgt, s, i) {
		if (sg_dma_address(s) != expected)
			return 0;
		expected += sg_dma_len(s);
	}
	return 1;
}

static int snps_accel_dmabuf_attach_device(struct dma_buf *dmabuf,
					   struct device *dev,
					   struct snps_accel_mem_buffer *mbuf,
					   enum dma_data_direction dma_dir)
{
	struct dma_buf_attachment *dba;
	struct sg_table *sgt;

	dba = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(dba))
		return PTR_ERR(dba);

	sgt = dma_buf_map_attachment(dba, dma_dir);
	if (IS_ERR(sgt)) {
		dma_buf_detach(dmabuf, dba);
		return PTR_ERR(sgt);
	}

	mbuf->da = sg_dma_address(sgt->sgl);
	mbuf->dmasgt = sgt;
	mbuf->import_attach = dba;

	return 0;
}

static void
snps_accel_dmabuf_detach_device(struct snps_accel_mem_buffer *mbuf)
{
	if (mbuf->dmasgt)
		dma_buf_unmap_attachment(mbuf->import_attach, mbuf->dmasgt,
					 mbuf->dma_dir);
	dma_buf_detach(mbuf->dmabuf, mbuf->import_attach);
	dma_buf_put(mbuf->dmabuf);
}

static void snps_accel_dmabuf_op_release(struct dma_buf *dmabuf)
{
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;
	struct snps_accel_mem_ctx *mem = mbuf->ctx;

	snps_accel_mbuf_free(mem, mbuf, mbuf->dma_dir);
}

static int
snps_accel_dmabuf_op_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;
	int ret = 0;

	ret = dma_mmap_pages(mbuf->dev, vma, mbuf->size, virt_to_page(mbuf->va));
	if (ret)
		return ret;

	return 0;
}

static int snps_accel_dmabuf_op_attach(struct dma_buf *dmabuf,
				       struct dma_buf_attachment *attachment)
{
	struct snps_accel_dmabuf_attachment *dba;
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;
	int ret;

	dba = kzalloc(sizeof(*dba), GFP_KERNEL);
	if (!dba)
		return -ENOMEM;

	ret = dma_get_sgtable(mbuf->dev, &dba->sgt, mbuf->va,
			      mbuf->da, mbuf->size);
	if (ret < 0) {
		dev_err(mbuf->dev, "Failed to get scatter list from DMA API\n");
		kfree(dba);
		return -EINVAL;
	}

	dba->dev = attachment->dev;
	INIT_LIST_HEAD(&dba->node);
	attachment->priv = dba;
	dba->mapped = false;

	mutex_lock(&mbuf->lock);
	list_add(&dba->node, &mbuf->attachments);
	mutex_unlock(&mbuf->lock);

	return 0;
}

static void snps_accel_dmabuf_op_detach(struct dma_buf *dmabuf,
					struct dma_buf_attachment *attachment)
{
	struct snps_accel_dmabuf_attachment *dba = attachment->priv;
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;

	mutex_lock(&mbuf->lock);
	list_del(&dba->node);
	mutex_unlock(&mbuf->lock);
	sg_free_table(&dba->sgt);
	kfree(dba);
}

static struct sg_table *
snps_accel_dmabuf_op_map(struct dma_buf_attachment *attachment,
			 enum dma_data_direction dir)
{
	struct snps_accel_dmabuf_attachment *dba = attachment->priv;
	struct sg_table *table;
	int ret;

	table = &dba->sgt;
	dba->mapped = true;

	ret = dma_map_sgtable(attachment->dev, table, dir, 0);
	if (ret)
		table = ERR_PTR(ret);

	return table;
}

static void snps_accel_dmabuf_op_unmap(struct dma_buf_attachment *attach,
				  struct sg_table *table,
				  enum dma_data_direction dir)
{
	struct snps_accel_dmabuf_attachment *dba = attach->priv;

	dba->mapped = false;
	dma_unmap_sgtable(attach->dev, table, dir, 0);
}

static int snps_accel_dmabuf_op_begin_cpu_access(struct dma_buf *dmabuf,
						 enum dma_data_direction direction)
{
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;
	struct snps_accel_dmabuf_attachment *dba;

	dma_sync_single_for_cpu(mbuf->dev, mbuf->da, mbuf->size, direction);

	mutex_lock(&mbuf->lock);
	list_for_each_entry(dba, &mbuf->attachments, node) {
		if (!dba->mapped)
			continue;
		dma_sync_sgtable_for_cpu(dba->dev, &dba->sgt, direction);
	}
	mutex_unlock(&mbuf->lock);

	return 0;
}

static int snps_accel_dmabuf_op_end_cpu_access(struct dma_buf *dmabuf,
					       enum dma_data_direction direction)
{
	struct snps_accel_mem_buffer *mbuf = dmabuf->priv;
	struct snps_accel_dmabuf_attachment *dba;

	dma_sync_single_for_device(mbuf->dev, mbuf->da, mbuf->size, direction);

	mutex_lock(&mbuf->lock);
	list_for_each_entry(dba, &mbuf->attachments, node) {
		if (!dba->mapped)
			continue;
		dma_sync_sgtable_for_device(dba->dev, &dba->sgt, direction);
	}
	mutex_unlock(&mbuf->lock);

	return 0;
}

static const struct dma_buf_ops snps_accel_dmabuf_ops = {
	.attach = snps_accel_dmabuf_op_attach,
	.detach = snps_accel_dmabuf_op_detach,
	.map_dma_buf = snps_accel_dmabuf_op_map,
	.unmap_dma_buf = snps_accel_dmabuf_op_unmap,
	.begin_cpu_access = snps_accel_dmabuf_op_begin_cpu_access,
	.end_cpu_access = snps_accel_dmabuf_op_end_cpu_access,
	.mmap = snps_accel_dmabuf_op_mmap,
	.release = snps_accel_dmabuf_op_release,
};

void snps_accel_app_mem_init(struct device *dev, struct snps_accel_mem_ctx *mem)
{
	mem->dev = dev;
	mutex_init(&mem->list_lock);
	INIT_LIST_HEAD(&mem->mlist);
	mem->num_regions = 0;
}

void snps_accel_app_mem_init_regions(struct snps_accel_mem_ctx *mem,
				     struct snps_accel_mem_region *regions,
				     u32 num_regions)
{
	int i;

	if (num_regions > SNPS_ACCEL_MAX_MEM_REGIONS) {
		dev_warn(mem->dev, "Too many regions %d, limiting to %d\n",
			 num_regions, SNPS_ACCEL_MAX_MEM_REGIONS);
		num_regions = SNPS_ACCEL_MAX_MEM_REGIONS;
	}

	for (i = 0; i < num_regions; i++)
		mem->regions[i] = regions[i];
	mem->num_regions = num_regions;

	dev_dbg(mem->dev, "Initialized memory context with %d region(s)\n", num_regions);
}

void snps_accel_app_release_import(struct snps_accel_mem_ctx *mem)
{
	struct snps_accel_mem_buffer *mbuf, *nmb;
	struct snps_accel_file_priv *fpriv = to_snps_accel_file_priv(mem);

	mutex_lock(&mem->list_lock);
	list_for_each_entry_safe(mbuf, nmb, &mem->mlist, ctx_link) {
		if (mbuf->imported) {
			list_del(&mbuf->ctx_link);
			snps_accel_dmabuf_detach_device(mbuf);
			kfree(mbuf);
			snps_accel_file_priv_put(fpriv);
		}
	}
	mutex_unlock(&mem->list_lock);
}

static inline enum dma_data_direction snps_accel_app_dma_direction(u32 dflags)
{
	unsigned ret = DMA_BIDIRECTIONAL;

	if (dflags == SNPS_ACCEL_IO_R)
		ret = DMA_FROM_DEVICE;
	if (dflags == SNPS_ACCEL_IO_W)
		ret = DMA_TO_DEVICE;

	return ret;
}

struct snps_accel_mem_buffer *snps_accel_app_dmabuf_create(struct snps_accel_mem_ctx *mem,
							   u64 size, u32 dflags)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct snps_accel_mem_buffer *mbuf;
	enum dma_data_direction dma_dir = snps_accel_app_dma_direction(dflags);

	/*
	 * Allocate backing memory and export it as a dma_buf. The caller is
	 * responsible for installing the dma_buf into a file descriptor
	 * (via get_unused_fd_flags() + fd_install() on mbuf->dmabuf->file).
	 */
	mbuf = snps_accel_mbuf_alloc(mem, size, dma_dir);
	if (mbuf == NULL)
		return NULL;

	exp_info.ops = &snps_accel_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.priv = mbuf;
	mbuf->dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(mbuf->dmabuf)) {
		dev_dbg(mem->dev, "Failed to create dmabuf\n");
		mbuf->dmabuf = NULL;
		snps_accel_mbuf_free(mem, mbuf, dma_dir);
		return NULL;
	}

	return mbuf;
}

int snps_accel_app_dmabuf_info(struct snps_accel_mem_ctx *mem,
			       struct snps_accel_dmabuf_info *info)
{
	struct dma_buf *dmabuf;
	struct snps_accel_mem_buffer *mbuf;
	int ret = -EINVAL;

	dmabuf = dma_buf_get(info->fd);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	mutex_lock(&mem->list_lock);
	mbuf = snps_accel_dmabuf_find_by_dmabuf_locked(mem, dmabuf);
	if (mbuf) {
		info->addr = mbuf->da;
		info->size = mbuf->size;
		dev_dbg(mbuf->dev,
			"dmabuf info: fd %d, va/pa/da %px/%pa/%pad, size %zu\n",
			info->fd, mbuf->va, &mbuf->pa, &mbuf->da, mbuf->size);
		ret = 0;
	}
	mutex_unlock(&mem->list_lock);

	if (ret)
		dev_err(mem->dev, "Failed to find dmabuf with fd %d\n", info->fd);

	dma_buf_put(dmabuf);
	return ret;
}

/*
 * Cache maintenance of one byte range of a dma-buf this driver exported, from
 * any process holding its fd (see struct snps_accel_sync_range). The
 * direction comes from the call, not from the buffer's allocation flags, as
 * for begin_cpu_access / end_cpu_access: what matters is who wrote last.
 */
int snps_accel_app_dmabuf_sync_range(const struct snps_accel_sync_range *r)
{
	const u32 valid = SNPS_ACCEL_SYNC_TO_DEVICE | SNPS_ACCEL_SYNC_TO_CPU;
	struct snps_accel_mem_buffer *mbuf;
	struct snps_accel_dmabuf_attachment *dba;
	struct dma_buf *dmabuf;
	bool attached = false;
	u64 end;
	int ret = 0;

	if (!r->flags || (r->flags & ~valid))
		return -EINVAL;

	/* holds a reference to the file, so the buffer stays alive */
	dmabuf = dma_buf_get(r->fd);
	if (IS_ERR(dmabuf))
		return -EBADF;

	/* only buffers this driver exported; an imported one belongs to its exporter */
	if (dmabuf->ops != &snps_accel_dmabuf_ops) {
		ret = -EINVAL;
		goto out;
	}
	mbuf = dmabuf->priv;

	if (check_add_overflow(r->offset, r->length, &end) || end > mbuf->size) {
		ret = -EINVAL;
		goto out;
	}
	if (!r->length)
		goto out;

	/*
	 * A buffer mapped by an importing device: its own mappings would need
	 * the same range; not handled, the caller syncs the whole buffer
	 * through DMA_BUF_IOCTL_SYNC instead.
	 */
	mutex_lock(&mbuf->lock);
	list_for_each_entry(dba, &mbuf->attachments, node)
		attached = attached || dba->mapped;
	mutex_unlock(&mbuf->lock);
	if (attached) {
		ret = -EBUSY;
		goto out;
	}

	/* clean first, then invalidate; both end with a DSB */
	if (r->flags & SNPS_ACCEL_SYNC_TO_DEVICE)
		dma_sync_single_range_for_device(mbuf->dev, mbuf->da, r->offset,
						 r->length, DMA_TO_DEVICE);
	if (r->flags & SNPS_ACCEL_SYNC_TO_CPU)
		dma_sync_single_range_for_cpu(mbuf->dev, mbuf->da, r->offset,
					      r->length, DMA_FROM_DEVICE);
out:
	dma_buf_put(dmabuf);
	return ret;
}

void snps_accel_app_dmabuf_release(struct snps_accel_mem_buffer *mbuf)
{
	dma_buf_put(mbuf->dmabuf);
}

int snps_accel_app_dmabuf_import(struct snps_accel_mem_ctx *mem, int fd)
{
	struct dma_buf *dmabuf;
	struct snps_accel_mem_buffer *mbuf;
	int ret;
	struct snps_accel_file_priv *fpriv = to_snps_accel_file_priv(mem);
	struct device *dmabuf_dev = mem->dev;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		dev_err(mem->dev, "Failed to get dma_buf with fd %d\n", fd);
		return -EINVAL;
	}

	mbuf = kzalloc(sizeof(*mbuf), GFP_KERNEL);
	if (!mbuf) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	mbuf->imported = true;
	mbuf->dma_dir = DMA_BIDIRECTIONAL;
	mbuf->dev = dmabuf_dev;
	ret = snps_accel_dmabuf_attach_device(dmabuf, mem->dev,
					      mbuf, mbuf->dma_dir);
	if (ret != 0) {
		dev_err(mem->dev, "Failed to attach dmabuf to device\n");
		goto err_attach;
	}

	if (!snps_accel_dmabuf_is_contig(mbuf->dmasgt)) {
		ret = -EINVAL;
		goto err_notcontig;
	}

	mbuf->dmabuf = dmabuf;
	mbuf->size = dmabuf->size;
	mbuf->va = NULL;

	mutex_lock(&mem->list_lock);
	list_add(&mbuf->ctx_link, &mem->mlist);
	mutex_unlock(&mem->list_lock);

	snps_accel_file_priv_get(fpriv);

	return 0;

err_notcontig:
	dma_buf_unmap_attachment(mbuf->import_attach, mbuf->dmasgt,
				 mbuf->dma_dir);
	dma_buf_detach(dmabuf, mbuf->import_attach);
err_attach:
	kfree(mbuf);
err_alloc:
	dma_buf_put(dmabuf);
	return ret;
}

int snps_accel_app_dmabuf_detach(struct snps_accel_mem_ctx *mem, int fd)
{
	struct snps_accel_mem_buffer *mbuf, *imported_mbuf = NULL;
	struct snps_accel_file_priv *fpriv = to_snps_accel_file_priv(mem);
	struct dma_buf *dmabuf;
	bool found;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		dev_err(mem->dev, "Failed to find imported dmabuf with fd %d\n", fd);
		return -EINVAL;
	}

	mutex_lock(&mem->list_lock);
	mbuf = snps_accel_dmabuf_find_by_dmabuf_locked(mem, dmabuf);
	found = (mbuf != NULL);
	if (mbuf && mbuf->imported) {
		list_del(&mbuf->ctx_link);
		imported_mbuf = mbuf;
	}
	mutex_unlock(&mem->list_lock);

	dma_buf_put(dmabuf);

	if (imported_mbuf) {
		snps_accel_dmabuf_detach_device(imported_mbuf);
		kfree(imported_mbuf);
		snps_accel_file_priv_put(fpriv);
		return 0;
	}

	if (found)
		dev_warn(mem->dev, "Detach on non-imported dmabuf fd %d\n", fd);

	return -EINVAL;
}
