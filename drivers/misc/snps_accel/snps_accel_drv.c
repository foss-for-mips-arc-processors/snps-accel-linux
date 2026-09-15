// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2025 Synopsys, Inc. (www.synopsys.com)
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#if IS_ENABLED(CONFIG_OF_IOMMU)
#include <linux/of_iommu.h>
#endif
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <uapi/misc/snps_accel.h>
#include "snps_accel_drv.h"

#define MAX_DEVS		32
#define DRIVER_NAME		"snps_accel_app"
#define DEV_NAME_FORMAT		"snps!arcnet%d!app%d"

static struct class *snps_accel_class;
static unsigned int snps_accel_major;

enum {
	snps_accel_pgprot_noncached,
	snps_accel_pgprot_writecombine = 1
};

static int
snps_accel_info_shmem(struct snps_accel_app *accel_app, char __user *argp)
{
	struct snps_accel_shmem data;

	data.offset = accel_app->shmem_base;
	data.size = accel_app->shmem_size;
	if (copy_to_user((void __user *)argp, &data,
			 sizeof(struct snps_accel_shmem)))
		return -EFAULT;

	return 0;
}

static int
snps_accel_info_notify(struct snps_accel_app *accel_app, char __user *argp)
{
	struct snps_accel_notify data;

	data.offset = accel_app->ctrl_base;
	data.size = accel_app->ctrl_size;
	if (copy_to_user((void __user *)argp, &data,
			 sizeof(struct snps_accel_notify)))
		return -EFAULT;

	return 0;
}

static int
snps_accel_info_cluster(struct snps_accel_app *accel_app, char __user *argp)
{
	struct snps_accel_cluster_info data = { };

	if (accel_app->virt_mode)
		data.flags |= SNPS_ACCEL_CLUSTER_FLAG_VIRT;
	data.host_cluster_id = accel_app->host_cluster_id;
	data.target_cluster_id = accel_app->target_cluster_id;

	if (copy_to_user((void __user *)argp, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static int
snps_accel_wait_irq(struct snps_accel_file_priv *fpriv, char __user *argp)
{
	struct snps_accel_app *accel_app = fpriv->app;
	struct snps_accel_wait_irq data;
	int ret = 0;
	long tret;
	u32 event_count;

	if (!accel_app || !accel_app->ctrl.dev || accel_app->irq_num < 0)
		return -EIO;

	if (copy_from_user(&data, (void __user *)argp,
			  sizeof(struct snps_accel_wait_irq)))
		return -EFAULT;

	event_count = atomic_read(&accel_app->irq_event);

	if (data.timeout != 0 && fpriv->handled_irq_event == event_count) {
		tret = wait_event_interruptible_timeout(accel_app->wait,
				atomic_read(&accel_app->irq_event) != fpriv->handled_irq_event,
				msecs_to_jiffies(data.timeout));
		/* Let the syscall be restarted/interrupted on a signal */
		if (tret < 0)
			return tret;
		if (tret == 0)
			ret = -ETIMEDOUT;
		event_count = atomic_read(&accel_app->irq_event);
	}

	fpriv->handled_irq_event = data.count = event_count;
	if (copy_to_user((void __user *)argp, &data, sizeof(struct snps_accel_wait_irq)))
		return -EFAULT;

	return ret;
}

static int
snps_accel_do_dmabuf_alloc(struct snps_accel_file_priv *fpriv, char __user *argp)
{
	struct snps_accel_dmabuf_alloc data;
	struct snps_accel_mem_buffer *mbuf;
	int fd;

	if (copy_from_user(&data, (void __user *)argp,
			  sizeof(struct snps_accel_dmabuf_alloc)))
		return -EFAULT;

	mbuf = snps_accel_app_dmabuf_create(&fpriv->mem, data.size, data.flags);
	if (!mbuf)
		return -ENOMEM;
	/*
	 * get_unused_fd_flags() + fd_install() are split so that all fallible
	 * work (including copy_to_user) runs before the fd becomes visible to
	 * userspace.
	 */
	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		snps_accel_app_dmabuf_release(mbuf);
		return fd;
	}

	data.fd = fd;
	if (copy_to_user((void __user *)argp, &data, sizeof(data))) {
		put_unused_fd(fd);
		snps_accel_app_dmabuf_release(mbuf);
		return -EFAULT;
	}

	fd_install(fd, mbuf->dmabuf->file);
	return 0;
}

static int
snps_accel_do_dmabuf_info(struct snps_accel_file_priv *fpriv, char __user *argp)
{
	struct snps_accel_dmabuf_info data;
	int ret;

	if (copy_from_user(&data, (void __user *)argp,
			  sizeof(struct snps_accel_dmabuf_info)))
		return -EFAULT;

	ret = snps_accel_app_dmabuf_info(&fpriv->mem, &data);
	if (ret)
		return ret;

	if (copy_to_user((void __user *)argp, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static int
snps_accel_do_dmabuf_import(struct snps_accel_file_priv *fpriv, char __user *argp)
{
	struct snps_accel_dmabuf_import data;
	int ret;

	if (copy_from_user(&data, (void __user *)argp,
			  sizeof(struct snps_accel_dmabuf_import)))
		return -EFAULT;

	ret = snps_accel_app_dmabuf_import(&fpriv->mem, data.fd);
	if (ret)
		return ret;

	return 0;
}

static int
snps_accel_do_dmabuf_detach(struct snps_accel_file_priv *fpriv, char __user *argp)
{
	struct snps_accel_dmabuf_detach data;
	int ret;

	if (copy_from_user(&data, (void __user *)argp,
			  sizeof(struct snps_accel_dmabuf_detach)))
		return -EFAULT;

	ret = snps_accel_app_dmabuf_detach(&fpriv->mem, data.fd);
	if (ret)
		return ret;

	return 0;
}

static void file_priv_release(struct kref *ref)
{
	struct snps_accel_file_priv *fpriv = container_of(ref, struct snps_accel_file_priv, ref);

	kfree(fpriv);
}

void snps_accel_file_priv_get(struct snps_accel_file_priv *fpriv)
{
	kref_get(&fpriv->ref);
}

void snps_accel_file_priv_put(struct snps_accel_file_priv *fpriv)
{
	kref_put(&fpriv->ref, file_priv_release);
}

static int snps_accel_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct snps_accel_file_priv *fpriv;
	struct snps_accel_app *accel_app =
		container_of(cdev, struct snps_accel_app, cdev);

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv)
		return -ENOMEM;

	kref_init(&fpriv->ref);
	fpriv->app = accel_app;
	snps_accel_app_mem_init(accel_app->device, &fpriv->mem);

	if (accel_app->num_mem_regions > 0) {
		snps_accel_app_mem_init_regions(&fpriv->mem,
						accel_app->mem_regions,
						accel_app->num_mem_regions);
	}

	fpriv->handled_irq_event = atomic_read(&accel_app->irq_event);
	filp->private_data = fpriv;

	return 0;
}

static int snps_accel_close(struct inode *inode, struct file *filp)
{
	struct snps_accel_file_priv *fpriv = (struct snps_accel_file_priv *)filp->private_data;

	flush_delayed_fput();
	snps_accel_app_release_import(&fpriv->mem);
	snps_accel_file_priv_put(fpriv);
	return 0;
}

static long
snps_accel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct snps_accel_file_priv *fpriv = (struct snps_accel_file_priv *)filp->private_data;
	struct snps_accel_app *accel_app = fpriv->app;
	char __user *argp = (char __user *)arg;
	int err;

	switch (cmd) {
	case SNPS_ACCEL_IOCTL_INFO_SHMEM:
		err = snps_accel_info_shmem(accel_app, argp);
		break;
	case SNPS_ACCEL_IOCTL_INFO_NOTIFY:
		err = snps_accel_info_notify(accel_app, argp);
		break;
	case SNPS_ACCEL_IOCTL_WAIT_IRQ:
		err = snps_accel_wait_irq(fpriv, argp);
		break;
	case SNPS_ACCEL_IOCTL_DMABUF_ALLOC:
		err = snps_accel_do_dmabuf_alloc(fpriv, argp);
		break;
	case SNPS_ACCEL_IOCTL_DMABUF_INFO:
		err = snps_accel_do_dmabuf_info(fpriv, argp);
		break;
	case SNPS_ACCEL_IOCTL_DMABUF_IMPORT:
		err = snps_accel_do_dmabuf_import(fpriv, argp);
		break;
	case SNPS_ACCEL_IOCTL_DMABUF_DETACH:
		err = snps_accel_do_dmabuf_detach(fpriv, argp);
		break;
	case SNPS_ACCEL_IOCTL_INFO_CLUSTER:
		err = snps_accel_info_cluster(accel_app, argp);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

static int snps_accel_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct snps_accel_file_priv *fpriv = (struct snps_accel_file_priv *)filp->private_data;
	struct snps_accel_app *accel_app = fpriv->app;
	int ret;
	u64 addr = vma->vm_pgoff << PAGE_SHIFT;
	size_t size = vma->vm_end - vma->vm_start;

	if (addr == accel_app->shmem_base) {
		if (size != accel_app->shmem_size && size != PAGE_SIZE) {
			dev_dbg(accel_app->device, "Shared memory size mismatch\n");
			return -EINVAL;
		}
		if (accel_app->pgprot_bits & snps_accel_pgprot_writecombine) {
			vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		}
		else {
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		}
		ret = remap_pfn_range(vma, vma->vm_start,
				      vma->vm_pgoff,
				      size,
				      vma->vm_page_prot);
	} else if (addr == accel_app->ctrl_base) {
		if (size != accel_app->ctrl_size && size != PAGE_SIZE) {
			dev_dbg(accel_app->device, "Notify memory size mismatch\n");
			return -EINVAL;
		}
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		ret = io_remap_pfn_range(vma, vma->vm_start,
					 vma->vm_pgoff,
					 size,
					 vma->vm_page_prot);
	} else {
		dev_dbg(accel_app->device, "Unsupported address to mmap %pap\n",
			&addr);
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations snps_accel_app_fops = {
	.owner		= THIS_MODULE,
	.open		= snps_accel_open,
	.release	= snps_accel_close,
	.unlocked_ioctl	= snps_accel_ioctl,
	.compat_ioctl	= snps_accel_ioctl,
	.mmap		= snps_accel_mmap,
};

static int
snps_accel_get_ctrl_mem(struct device_node *node, struct resource *ctrl)
{
	int ret;
	struct device_node *np;

	/* Get control unit reference */
	np = of_parse_phandle(node, "snps,arcsync-ctrl", 0);
	if (!np)
		return -EINVAL;

	/* Get control unit registers base address */
	ret = of_address_to_resource(np, 0, ctrl);
	of_node_put(np);
	if (ret < 0)
		return ret;

	return 0;
}

static struct device_node *
snps_accel_of_get_main_rproc(struct device_node *node)
{
	struct device_node *rproc_np;

	if (!of_find_property(node, "snps,main-rproc", NULL))
		return NULL;

	rproc_np = of_parse_phandle(node, "snps,main-rproc", 0);
	if (!rproc_np)
		return ERR_PTR(-EINVAL);

	if (!of_device_is_compatible(rproc_np, "snps,npx-rproc") &&
	    !of_device_is_compatible(rproc_np, "snps,vpx-rproc")) {
		of_node_put(rproc_np);
		return ERR_PTR(-EINVAL);
	}

	return rproc_np;
}

static u32 snps_accel_of_get_rproc_target_id(struct snps_accel_app *accel_app,
					     struct device_node *rproc_np)
{
	u32 clid = 0;
	u32 cid = 0;

	if (!rproc_np)
		return 0;

	of_property_read_u32(rproc_np, "snps,arcsync-cluster-id", &clid);
	if (accel_app->virt_mode)
		return clid;

	of_property_read_u32_index(rproc_np, "snps,arcsync-core-id", 0, &cid);

	return accel_app->ctrl.fn.build_coreid(accel_app->ctrl.dev, clid, cid);
}

static void snps_accel_memdev_release(struct device *dev)
{
	kfree(dev);
}

static struct device *snps_accel_alloc_mem_device(struct device *parent, int idx)
{
	struct device *child;
	int ret;

	child = kzalloc(sizeof(*child), GFP_KERNEL);
	if (!child)
		return NULL;

	device_initialize(child);
	dev_set_name(child, "%s:mem%d", dev_name(parent), idx);
	child->parent = parent;
	child->coherent_dma_mask = parent->coherent_dma_mask;
	child->dma_mask = &child->coherent_dma_mask;
	child->release = snps_accel_memdev_release;

	ret = device_add(child);
	if (ret) {
		put_device(child);
		return NULL;
	}

	ret = of_dma_configure(child, parent->of_node, true);
	if (ret) {
		device_unregister(child);
		return NULL;
	}

	return child;
}

static int snps_accel_init_mem_regions(struct snps_accel_app *accel_app,
				       struct device_node *node)
{
	struct device *parent_dev = accel_app->device;
	int num_regions, i, ret;

	num_regions = of_count_phandle_with_args(node, "memory-region", NULL);
	if (num_regions <= 0) {
		dev_dbg(parent_dev, "No memory-region specified, using device default\n");
		accel_app->num_mem_regions = 0;
		return 0;
	}

	/* With an IOMMU enabled, do not create memory-region child devices.
	 * Bind only the first memory region to the accelerator device, or
	 * fall back to system memory allocations if no memory region is
	 * specified.
	 */
	if (accel_app->iommu_backed && num_regions > 1) {
		dev_warn(parent_dev,
			 "IOMMU mode: %d memory-regions found, using only the first\n",
			 num_regions);
		num_regions = 1;
	}

	if (num_regions == 1) {
		ret = of_reserved_mem_device_init_by_idx(parent_dev, node, 0);
		if (ret) {
			dev_warn(parent_dev,
				 "Failed to bind reserved region to app device (%d)\n",
				 ret);
			accel_app->num_mem_regions = 0;
		} else {
			dev_info(parent_dev,
				 "Reserved memory region bound to app device\n");
			accel_app->num_mem_regions = 1;
		}

		return 0;
	}

	if (num_regions > SNPS_ACCEL_MAX_MEM_REGIONS) {
		dev_warn(parent_dev, "Too many memory regions (%d), using first %d\n",
			 num_regions, SNPS_ACCEL_MAX_MEM_REGIONS);
		num_regions = SNPS_ACCEL_MAX_MEM_REGIONS;
	}

	for (i = 0; i < num_regions; i++) {
		struct device *child;
		struct device_node *mem_node;
		u64 base = 0;
		u64 size = 0;

		/* Create child device */
		child = snps_accel_alloc_mem_device(parent_dev, i);
		if (!child) {
			dev_err(parent_dev, "Failed to allocate mem device %d\n", i);
			ret = -ENOMEM;
			goto err_cleanup;
		}

		ret = of_reserved_mem_device_init_by_idx(child, node, i);
		if (ret) {
			dev_err(parent_dev, "Failed to init reserved mem for region %d: %d\n",
				i, ret);
			device_unregister(child);
			goto err_cleanup;
		}

		mem_node = of_parse_phandle(node, "memory-region", i);
		if (mem_node) {
			struct reserved_mem *rmem = of_reserved_mem_lookup(mem_node);

			if (rmem) {
				base = rmem->base;
				size = rmem->size;
			}
			of_node_put(mem_node);
		}

		if (!size)
			dev_warn(parent_dev, "Region %d range unknown, confinement disabled\n", i);

		accel_app->mem_regions[i].dev = child;
		accel_app->mem_regions[i].base = base;
		accel_app->mem_regions[i].size = size;

		dev_info(parent_dev, "Memory region %d: device %s, base 0x%llx size %llu bytes (0x%llx)\n",
			 i, dev_name(child), base, size, size);
	}

	accel_app->num_mem_regions = num_regions;
	return 0;

err_cleanup:
	while (--i >= 0) {
		of_reserved_mem_device_release(accel_app->mem_regions[i].dev);
		device_unregister(accel_app->mem_regions[i].dev);
	}
	accel_app->num_mem_regions = 0;
	return ret;
}

static void snps_accel_release_mem_regions(struct snps_accel_app *accel_app)
{
	u32 i;

	if (accel_app->num_mem_regions == 1) {
		of_reserved_mem_device_release(accel_app->device);
		accel_app->num_mem_regions = 0;
		return;
	}

	for (i = 0; i < accel_app->num_mem_regions; i++) {
		if (accel_app->mem_regions[i].dev) {
			of_reserved_mem_device_release(accel_app->mem_regions[i].dev);
			device_unregister(accel_app->mem_regions[i].dev);
			accel_app->mem_regions[i].dev = NULL;
		}
	}
	accel_app->num_mem_regions = 0;
}

static irqreturn_t snps_accel_app_irq_callback(int irq, void *dev)
{
	struct snps_accel_app *accel_app = dev;

	atomic_inc(&accel_app->irq_event);
	wake_up_interruptible(&accel_app->wait);

	return IRQ_HANDLED;
}

static int
snps_accel_init_ctrl_with_arcsync_fn(struct snps_accel_app *accel_app, struct device *arcsync_dev)
{
	const struct arcsync_funcs *arcsync_fn;
	struct snps_accel_ctrl_fn *ctrl_fn = &accel_app->ctrl.fn;

	arcsync_fn = arcsync_get_ctrl_fn(arcsync_dev);
	if (IS_ERR(arcsync_fn))
		return PTR_ERR(arcsync_fn);

	ctrl_fn->set_interrupt_callback = arcsync_fn->set_interrupt_callback;
	ctrl_fn->remove_interrupt_callback = arcsync_fn->remove_interrupt_callback;
	ctrl_fn->build_coreid = arcsync_fn->build_coreid;

	accel_app->ctrl.arcnet_id = arcsync_fn->get_arcnet_id(arcsync_dev);
	accel_app->virt_mode = arcsync_fn->get_virt_mode(arcsync_dev);
	accel_app->host_cluster_id = arcsync_fn->get_host_cluster_id(arcsync_dev);

	return 0;
}

static int
snps_accel_add_app(struct platform_device *pdev, struct device_node *node)
{
	int ret;
	struct snps_accel_app *accel_app;
	struct snps_accel_device *accel_dev = dev_get_drvdata(&pdev->dev);
	struct resource ctrl;
	struct resource shmem;
	struct device_node *rproc_np;
	u32 dma_bits = 32;
	u32 pgprot_bits;

	ret = snps_accel_get_ctrl_mem(node, &ctrl);
	if (ret < 0) {
		dev_err(&pdev->dev, "ARCsync control unit MMIO is not found\n");
		/* Return 0 to skip this app */
		return 0;
	}

	ret = of_address_to_resource(node, 0, &shmem);
	if (ret < 0) {
		dev_err(&pdev->dev, "Shared memory is not found\n");
		/* Return 0 to skip this app */
		return 0;
	}

	accel_app = kzalloc(sizeof(*accel_app), GFP_KERNEL);
	if (!accel_app)
		return -ENOMEM;

	/* Get ARCsync device reference and init ctrl func struct with arcsync funcs */
	accel_app->ctrl.dev = arcsync_get_device_by_phandle(node, "snps,arcsync-ctrl");
	if (IS_ERR(accel_app->ctrl.dev)) {
		dev_err(&pdev->dev, "Failed to get ARCSync ref: %ld\n",
			PTR_ERR(accel_app->ctrl.dev));

		ret = PTR_ERR(accel_app->ctrl.dev);
		goto err_get_arcsync_dev;
	}
	ret = snps_accel_init_ctrl_with_arcsync_fn(accel_app, accel_app->ctrl.dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get ARCSync funcs\n");
		goto err_get_arcsync_dev;
	}

	rproc_np = snps_accel_of_get_main_rproc(node);
	if (IS_ERR(rproc_np)) {
		dev_err(&pdev->dev, "Invalid snps,main-rproc property\n");
		ret = PTR_ERR(rproc_np);
		goto err_get_arcsync_dev;
	}
	accel_app->target_cluster_id =
		snps_accel_of_get_rproc_target_id(accel_app, rproc_np);
	of_node_put(rproc_np);

	cdev_init(&accel_app->cdev, &snps_accel_app_fops);
	accel_app->cdev.owner = THIS_MODULE;
	ret = cdev_add(&accel_app->cdev,
		       MKDEV(snps_accel_major, accel_dev->minor_count), MAX_DEVS);
	if (ret)
		goto err_cdev_add;

	accel_app->devt = MKDEV(snps_accel_major, accel_dev->minor_count);
	accel_app->device = device_create(snps_accel_class, &pdev->dev,
					  accel_app->devt,
					  accel_app,
					  DEV_NAME_FORMAT,
					  accel_app->ctrl.arcnet_id,
					  accel_dev->minor_count);
	if (IS_ERR(accel_app->device)) {
		dev_err(&pdev->dev, "Failed to create device /dev/snps/arcnet%d/hw%d\n",
			accel_app->ctrl.arcnet_id, accel_dev->minor_count);
		ret = PTR_ERR(accel_app->device);
		goto err_dev_create;
	}

	accel_app->device->bus = pdev->dev.bus;
	accel_app->device->of_node = node;

	accel_app->device->coherent_dma_mask = pdev->dev.coherent_dma_mask;
	accel_app->device->dma_mask = &accel_app->device->coherent_dma_mask;
	ret = of_property_read_u32(node, "snps,dma-bits", &dma_bits);
	if (ret) {
		dev_warn(accel_app->device, "snps,dma-bits DTS property read error %d, use %u\n",
				ret, dma_bits);
	}
	ret = dma_set_coherent_mask(accel_app->device, DMA_BIT_MASK(dma_bits));
	if (ret) {
		dev_err(accel_app->device, "No suitable coherent DMA available\n");
		goto err_app_dev_init;
	}
	ret = dma_set_mask(accel_app->device, DMA_BIT_MASK(dma_bits));
	if (ret) {
		dev_err(accel_app->device, "No suitable DMA available\n");
		goto err_app_dev_init;
	}
	dev_dbg(accel_app->device, "dma mask 0x%llx, coherent 0x%llx\n",
			accel_app->device->dma_mask ? *accel_app->device->dma_mask : 0,
			accel_app->device->coherent_dma_mask);

	ret = of_property_read_u32(node, "snps,pgprot-bits", &pgprot_bits);
	if (!ret)
		accel_app->pgprot_bits = pgprot_bits;
	else
		accel_app->pgprot_bits = snps_accel_pgprot_writecombine;

	dev_dbg(accel_app->device, "shmem pgprot 0x%x\n", accel_app->pgprot_bits);
	dev_dbg(accel_app->device, "ARCSync %s host id %u target id %u\n",
		accel_app->virt_mode ? "virt" : "physical",
		accel_app->host_cluster_id, accel_app->target_cluster_id);

	/*
	 * The app devices are not proper OF platform devices. Apply the DMA
	 * configuration explicitly.
	 */
	ret = of_dma_configure(accel_app->device, node, true);
	if (ret < 0)
		dev_warn(accel_app->device, "Failed to configure DMA/IOMMU (err: %d)\n", ret);
	else
		dev_info(accel_app->device, "IOMMU/DMA configured successfully\n");

	accel_app->iommu_backed = device_iommu_mapped(accel_app->device);
	dev_info(accel_app->device, "DMA buffers are %s\n",
		 accel_app->iommu_backed ? "IOMMU-translated" :
					   "physically addressed");

	ret = snps_accel_init_mem_regions(accel_app, node);
	if (ret) {
		dev_err(accel_app->device, "Failed to initialize memory regions: %d\n", ret);
		goto err_app_dev_init;
	}

	/* Add interrupt callback for ARCSync interrupt */
	accel_app->irq_num = of_irq_get(node, 0);
	if (accel_app->irq_num >= 0) {
		ret = accel_app->ctrl.fn.set_interrupt_callback(accel_app->ctrl.dev,
					accel_app->irq_num,
					snps_accel_app_irq_callback, accel_app);
		if (!ret) {
			init_waitqueue_head(&accel_app->wait);
			dev_dbg(accel_app->device, "App IRQ: %d\n", accel_app->irq_num);
		} else {
			dev_warn(accel_app->device, "Not ARCSync IRQ %d\n", accel_app->irq_num);
			accel_app->irq_num = -EINVAL;
		}
	} else {
		dev_warn(accel_app->device, "Notification IRQ not specified\n");
	}

	accel_app->ctrl_base = ctrl.start;
	accel_app->ctrl_size = resource_size(&ctrl);
	accel_app->shmem_base = shmem.start;
	accel_app->shmem_size = resource_size(&shmem);

	dev_dbg(accel_app->device, "Control region: start %pap size %pap\n",
		&accel_app->ctrl_base, &accel_app->ctrl_size);
	dev_dbg(accel_app->device, "Shared region: start %pap size %pap\n",
		&accel_app->shmem_base, &accel_app->shmem_size);

	accel_dev->minor_count++;
	list_add_tail(&accel_app->link, &accel_dev->devs_list);

	return 0;

err_app_dev_init:
	device_destroy(snps_accel_class, accel_app->devt);
err_dev_create:
	cdev_del(&accel_app->cdev);
err_cdev_add:
err_get_arcsync_dev:
	kfree(accel_app);
	return ret;
}

static int snps_accel_create_devs(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = pdev->dev.of_node;

	do {
		node = of_find_compatible_node(node, NULL, "snps,accel-app");
		if (node) {
			ret = snps_accel_add_app(pdev, node);
			if (ret) {
				of_node_put(node);
				return ret;
			}
		}
	} while (node);

	return 0;
}

static void snps_accel_release_app(struct snps_accel_app *accel_app)
{
	const struct snps_accel_ctrl_fn *fn = &accel_app->ctrl.fn;

	if (accel_app->irq_num >= 0)
		fn->remove_interrupt_callback(accel_app->ctrl.dev,
					      accel_app->irq_num, accel_app);

	snps_accel_release_mem_regions(accel_app);
	device_destroy(snps_accel_class, accel_app->devt);
	cdev_del(&accel_app->cdev);
}

static void snps_accel_release_devs(struct platform_device *pdev)
{
	struct snps_accel_device *accel_dev = dev_get_drvdata(&pdev->dev);
	struct snps_accel_app *cur, *n;

	list_for_each_entry_safe(cur, n, &accel_dev->devs_list, link) {
		if (cur->device)
			snps_accel_release_app(cur);

		list_del(&cur->link);
		kfree(cur);
	}
}

static int snps_accel_probe(struct platform_device *pdev)
{
	struct snps_accel_device *accel_dev;
	struct resource *res;
	int ret;

	accel_dev = devm_kzalloc(&pdev->dev, sizeof(*accel_dev), GFP_KERNEL);
	if (!accel_dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&accel_dev->devs_list);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Shared memory is not defined\n");
		return -EINVAL;
	}
	accel_dev->shared_base = res->start;
	accel_dev->shared_size = resource_size(res);

	dev_dbg(&pdev->dev, "shared memory start %pa, size %pa\n",
			&accel_dev->shared_base, &accel_dev->shared_size);

	dev_set_drvdata(&pdev->dev, accel_dev);
	ret = snps_accel_create_devs(pdev);
	if (ret != 0) {
		snps_accel_release_devs(pdev);
		return ret;
	}

	return ret;
}

static int snps_accel_remove(struct platform_device *pdev)
{
	snps_accel_release_devs(pdev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id snps_accel_match[] = {
	{ .compatible = "snps,accel" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, snps_accel_match);
#endif

static struct platform_driver snps_accel_platform_driver = {
	.probe = snps_accel_probe,
	.remove = snps_accel_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(snps_accel_match),
	},
};

static int __init snps_accel_init(void)
{
	int ret;
	dev_t dev;

	snps_accel_class = class_create("snps-accel");
	if (IS_ERR(snps_accel_class)) {
		ret = PTR_ERR(snps_accel_class);
		goto err_class;
	}

	ret = alloc_chrdev_region(&dev, 0, MAX_DEVS, DRIVER_NAME);
	if (ret)
		goto err_chr;

	snps_accel_major = MAJOR(dev);

	ret = platform_driver_register(&snps_accel_platform_driver);
	if (ret < 0)
		goto err_reg;

	return 0;

err_reg:
	unregister_chrdev_region(dev, MAX_DEVS);
err_chr:
	class_destroy(snps_accel_class);
err_class:
	return ret;
}
late_initcall(snps_accel_init);

static void __exit snps_accel_exit(void)
{
	platform_driver_unregister(&snps_accel_platform_driver);
	unregister_chrdev_region(MKDEV(snps_accel_major, 0), MAX_DEVS);
	class_destroy(snps_accel_class);
}
module_exit(snps_accel_exit);

MODULE_AUTHOR("Synopsys Inc.");
MODULE_DESCRIPTION("NPX/VPX driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(DMA_BUF);
