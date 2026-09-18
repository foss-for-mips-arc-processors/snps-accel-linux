// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synopsys ARCsync Driver
 *
 * ARCsync - small module for synchronization and control of multiple
 * ARC processors assembled in a heterogeneous sub-system.
 *
 * Copyright (C) 2023 Synopsys, Inc. (www.synopsys.com)
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/snps_arcsync.h>

#ifdef CONFIG_ISA_ARCV2
#include <soc/arc/aux.h>

#define ARC_AUX_IDENTITY		0x004
#define ARC_AUX_CLUSTER_ID		0x298
#endif

/* ARCsync config params */
#define ARCSYNC_NUM_CLUSTERS		arcsync->clusters_num
#define ARCSYNC_MAX_COREID		arcsync->cores_max

/* ARCsync registers offsets */
#define ARCSYNC_BLD_CFG			0x0
#define ARCSYNC_NUM_CORE_CL0_3		0x4
#define ARCSYNC_NUM_CORE_CL4_7		0x8

#define ARCSYNC_BLD_VERSION_MASK	0xFF
#define ARCSYNC_BLD_CUSTERS_NUM(bcr)	((((bcr) >> 8) & 0xFF) + 1)
#define ARCSYNC_BLD_CORES_PER_CL(bcr)	(4 << (((bcr) >> 16) & 0x7))

#define ARCSYNC_BLD_AC_BITS(bcr)	(((bcr) >> 19) & 0x7)
#define ARCSYNC_BLD_AC_NUM(bcr)					\
({								\
	u32 __ac = ARCSYNC_BLD_AC_BITS(bcr);			\
	__ac ? 16U << __ac : 0U;				\
})

#define ARCSYNC_BLD_HAS_PMU		(1 << 22)
#define ARCSYNC_BLD_NUM_VMS(bcr)	((((bcr) >> 25) & 0x3) << 2)

/* ARCsync v1 definitions */
#define ARCSYNC1_CORE_CONTROL		0x1000
#define ARCSYNC1_CORE_RUN(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 (coreid) * 4)
#define ARCSYNC1_CORE_HALT(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x04 + (coreid) * 4)
#define ARCSYNC1_CORE_IVT_LO(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x08 + (coreid) * 4)
#define ARCSYNC1_CORE_IVT_HI(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x0C + (coreid) * 4)
#define ARCSYNC1_CORE_STATUS(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x10 + (coreid) * 4)
#define ARCSYNC1_CORE_RESET(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x14 + (coreid) * 4)
#define ARCSYNC1_CORE_PMODE(coreid)	(ARCSYNC1_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x18 + (coreid) * 4)

#define ARCSYNC1_RESET_PWD		0x5A5A0000

#define ARCSYNC1_CORE_POWERUP		0x1
#define ARCSYNC1_CORE_POWERDOWN		0x2

/* ARCsync v2 definitions */
#define ARCSYNC2_CL_CONTROL		0x1000
#define ARCSYNC2_CL_ENABEL(clid)	(ARCSYNC2_CL_CONTROL + \
					 (clid) * 4)
#define ARCSYNC2_CL_GRP_CLK_EN(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x04 + clid * 4)
#define ARCSYNC2_CL_GRP_RST(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x08 + clid * 4)
#define ARCSYNC2_CL_GRP0_PMOD(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x2C + clid * 4)
#define ARCSYNC2_CL_GRP1_PMOD(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x30 + clid * 4)
#define ARCSYNC2_CL_GRP2_PMOD(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x34 + clid * 4)
#define ARCSYNC2_CL_GRP3_PMOD(clid)	(ARCSYNC2_CL_CONTROL + \
					 ARCSYNC_NUM_CLUSTERS * 0x38 + clid * 4)

#define ARCSYNC2_CORE_CONTROL		0x2000
#define ARCSYNC2_CORE_PMODE(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 (coreid) * 4)
#define ARCSYNC2_CORE_RUN(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x04 + (coreid) * 4)
#define ARCSYNC2_CORE_HALT(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x08 + (coreid) * 4)
#define ARCSYNC2_CORE_IVT_LO(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x0C + (coreid) * 4)
#define ARCSYNC2_CORE_IVT_HI(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x10 + (coreid) * 4)
#define ARCSYNC2_CORE_STATUS(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x14 + (coreid) * 4)
#define ARCSYNC2_CORE_RESET(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x18 + (coreid) * 4)
#define ARCSYNC2_CORE_CLK_EN(coreid)	(ARCSYNC2_CORE_CONTROL + \
					 ARCSYNC_MAX_COREID * 0x1C + (coreid) * 4)

#define ARCSYNC2_RESET_ASSERT		0x5A5A0000
#define ARCSYNC2_RESET_DEASSERT		0xA5A50000

#define ARCSYNC2_CORE_POWERUP		0x0
#define ARCSYNC2_CORE_POWERDOWN		0x1

#define ARCSYNC_CORE_STATUS_HALT	0x1
#define ARCSYNC_CORE_STATUS_SLEEP	0x4
#define ARCSYNC_CORE_STATUS_PWDOWN_M1	0x40
#define ARCSYNC_CORE_STATUS_PWDOWN_M2	0x80

#define ARCSYNC2_GRP_POWERUP		0x0
#define ARCSYNC2_GRP_POWERDOWN		0x1

#define ARCSYNC2_GRP_CLK_PWD_DIS	0x5A5A0000
#define ARCSYNC2_GRP_CLK_PWD_EN		0xA5A50000

#define ARCSYNC2_EID			0x4000
#define ARCSYNC2_EID_RISE_IRQ(coreid, idx)		\
					(ARCSYNC2_EID + \
					 ARCSYNC_MAX_COREID * (0x10 + (idx) * 8) + \
					 (coreid) * 4)
#define ARCSYNC2_EID_ACK_IRQ(coreid, idx)		\
					(ARCSYNC2_EID + \
					 ARCSYNC_MAX_COREID * (0x14 + (idx) * 8) + \
					 (coreid) * 4)

#define ARCSYNC2_AC_NUM			arcsync->ac_num
#define ARCSYNC2_AC_CONTROL		0x9000
#define ARCSYNC2_AC_SIZE		(4 * ARCSYNC_MAX_COREID * ARCSYNC2_AC_NUM)
#define ARCSYNC2_AC_SIZE_ALIGNED	ALIGN(ARCSYNC2_AC_SIZE, 4096)

#define ARCSYNC2_VM_MAP_OFFSET		(ARCSYNC2_AC_CONTROL + \
					 ARCSYNC2_AC_SIZE_ALIGNED)
#define ARCSYNC2_VM_MAP_SIZE		0x1000
#define ARCSYNC2_VM_MAP_VALID		(1 << 31)
#define ARCSYNC2_VM_BASE_OFFSET		(ARCSYNC2_VM_MAP_OFFSET + \
					 ARCSYNC2_VM_MAP_SIZE)
#define ARCSYNC2_VM_SIZE		0x1000

#define ARCSYNC2_VM_EID_RAISE_IRQ(vm, irq, vp) \
					(ARCSYNC2_VM_BASE_OFFSET + \
					 (vm) * ARCSYNC2_VM_SIZE + \
					 (16 + (irq) * 8) * ARCSYNC_NUM_CLUSTERS + \
					 (vp) * 4)

#define ARCSYNC2_VM_EID_ACK_IRQ(vm, irq, vp) \
					(ARCSYNC2_VM_BASE_OFFSET + \
					 (vm) * ARCSYNC2_VM_SIZE + \
					 (20 + (irq) * 8) * ARCSYNC_NUM_CLUSTERS + \
					 (vp) * 4)

#define ARCSYNC_DEF_HOST_CLUSTER_ID	0x2

struct arcsync_device;

/**
 * struct arcsync_callback - element of callbacks list for each IRQ
 * @link - list of callbacks
 * @func - callback function registered by the external driver for this interrupt
 * @data - pointer with data for callback function
 */
struct arcsync_callback {
	struct list_head link;
	intr_callback_t func;
	void *data;
};

/**
 * struct arcsync_interrupt - describes each ARCSync interrupt
 * @arcsync - pointer to the arcsync device structure
 * @irqnum - described ARCSync interrupt number
 * @idx - ARCSync interrupt line index and index in the array of interrupt structs (0,1,2...)
 * @vm_id - VM this interrupt belongs to (virtual IRQ mode only)
 * @vm_irq_idx - interrupt index within the VM (virtual IRQ mode only)
 * @ack_offs - precomputed EID ACK register offset
 * @callbacks_list_lock - spinlock for the list of IRQ callbacks
 * @callbacks_list - the list of IRQ callbacks
 */
struct arcsync_interrupt {
	struct arcsync_device *arcsync;
	u32 irqnum;
	u32 idx;
	u32 vm_id;
	u32 vm_irq_idx;
	u32 ack_offs;
	char name[32];
	spinlock_t callbacks_list_lock;
	struct list_head callbacks_list;
};

/**
 * struct arcsync_device - arcsync device structure
 * @dev: driver model representation of the device
 * @regs: ARCsync control registers virtual base address
 * @version: ARCsync IP version
 * @corenum_width: width of corenum field in bits to count core id
 * @has_pmu: PMU presence flag
 * @clusters_num: number of clusters controlled by ARCsync
 * @ac_num: number of ARCSync atomic counters
 * @vm_num: number of virtual machines
 * @cores_max: number of cores controlled by ARCsync
 * @host_id: host CPU id as it seen by the ARCSync
 * @vdk_fix: use of VDK fix flag
 * @virt_irq: use virtualized (per-VM) interrupt ack path
 * @lock: lock for access to the ARCScyn MMIO
 * @num_irqs: number of ARCSync interrupts to handle
 * @irq: array of ARCsync host IRQs for notifications
 * @funcs: pointer to the structure with ARCSync control funcs
 */
struct arcsync_device {
	struct device *dev;
	void __iomem *regs;
	u32 version;
	u32 corenum_width;
	u32 has_pmu;
	u32 clusters_num;
	u32 cores_max;
	u32 ac_num;
	u32 vm_num;
	u32 host_id;
	u32 vdk_fix;
	bool virt_irq;
	u32 arcnet_id;
	struct mutex lock;
	u32 num_irqs;
	struct arcsync_interrupt irq[ARCSYNC_HOST_MAX_IRQS];
	const struct arcsync_funcs *funcs;
};

/**
 * struct arcsync_dev_data - per-compatible configuration from the match table
 * @virt_irq: device uses the virtualized (per-VM) interrupts
 */
struct arcsync_dev_data {
	bool virt_irq;
};

static struct platform_driver snps_arcsync_platform_driver;

static inline u32 arcsync_build_coreid(u32 clid, u32 cid, u32 width)
{
	return (clid << width) | cid;
}

static u32 arcsync_build_coreid_fn(struct device *dev, u32 clid, u32 cid)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync_build_coreid(clid, cid, arcsync->corenum_width);
}

/**
 * arcsync_version() - get ARCSync IP version
 * @dev: arcsync device handle
 *
 * Return ARCSync IP unit version, the driver reads the version from
 * the ARCSync build config register.
 *
 * Return: version number
 */
static int arcsync_version(struct device *dev)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync->version;
}

/**
 * arcsync_has_pmu() - get ARCSync has_pmu flag
 * @dev: arcsync device handle
 *
 * Returns a flag indicating the presence of a PMU module in ARCSync,
 * the driver reads the ARCSync build configuration register to
 * determine PMU presence.
 *
 * Return: 0 if no PMU or 1 if ARCSync has PMU
 */
static int arcsync_has_pmu(struct device *dev)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync->has_pmu;
}

/**
 * arcsync_arcnet_id() - get the logical index of ARCSync device
 * @dev: arcsync device handle
 *
 * Returns ARCSync device logical index. The driver reads index from the Device
 * Tree snps,arcnet-id property, default value 0.
 *
 * Return: arcsync index value
 */
static int arcsync_arcnet_id(struct device *dev)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync->arcnet_id;
}

/**
 * arcsync_virt_mode() - get virtualized interrupt mode flag
 * @dev: arcsync device handle
 *
 * Return: 1 if the virtualized interrupt path is used, 0 otherwise
 */
static int arcsync_virt_mode(struct device *dev)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync->virt_irq;
}

/**
 * arcsync_host_cluster_id() - get host cluster ID
 * @dev: arcsync device handle
 *
 * Return: host cluster/VP ID
 */
static int arcsync_host_cluster_id(struct device *dev)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	return arcsync->host_id;
}

/**
 * arcsync_clk_ctrl() - core clock enable/disable control
 * @dev: arcsync device handle
 * @clid: cluster number
 * @cid: core number inside cluster
 * @cmd: clock control command ARCSYNC_CLK_DIS or ARCSYNC_CLK_EN
 *
 * Enable or disable core clock.
 *
 * Return: 0
 */
static int arcsync_clk_ctrl(struct device *dev, u32 clid, u32 cid, u32 cmd)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);

	if (arcsync->version == 1)
		return 0;

	mutex_lock(&arcsync->lock);
	if (cmd == ARCSYNC_CLK_DIS)
		writel(ARCSYNC_CLK_DIS, arcsync->regs +  ARCSYNC2_CORE_CLK_EN(coreid));
	else
		writel(ARCSYNC_CLK_EN, arcsync->regs + ARCSYNC2_CORE_CLK_EN(coreid));
	mutex_unlock(&arcsync->lock);

	return 0;
}

/**
 * arcsync_power_ctrl() - core power control
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @cid: core number inside cluster
 * @cmd: power control command ARCSYNC_POWER_UP or ARCSYNC_POWER_DOWN
 *
 * Set core power UP or power DOWN state.
 *
 * Return: 0 on success or negative errno on failure.
 */
static int arcsync_power_ctrl(struct device *dev, u32 clid, u32 cid, u32 cmd)
{
	u32 power_cmd;
	u32 count = 10;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reg_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_PMODE(coreid) :
						   ARCSYNC1_CORE_PMODE(coreid);

	if (cmd == ARCSYNC_POWER_UP)
		power_cmd = (arcsync->version >= 2) ? ARCSYNC2_CORE_POWERUP :
						      ARCSYNC1_CORE_POWERUP;
	else
		power_cmd = (arcsync->version >= 2) ? ARCSYNC2_CORE_POWERDOWN :
						      ARCSYNC1_CORE_POWERDOWN;

	mutex_lock(&arcsync->lock);
	/* Ensure power up/down handshake is not running */
	while (readl(arcsync->regs + reg_offset) && --count)
		udelay(1);
	if (count)
		writel(power_cmd, arcsync->regs + reg_offset);
	mutex_unlock(&arcsync->lock);

	return count ? 0 : -EBUSY;
}

/**
 * arcsync_reset() - send a reset signal to the specified core
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @cid: core number inside cluster
 * @cmd: reset command ARCSYNC_RESET_DEASSERT or ARCSYNC_RESET_ASSERT
 *
 * Assert or de-assert the core reset line
 *
 * Return: 0 on success or negative errno on failure.
 */
static int arcsync_reset(struct device *dev, u32 clid, u32 cid, u32 cmd)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reg_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_RESET(coreid) :
						   ARCSYNC1_CORE_RESET(coreid);
	u32 pwd;

	if (cmd == ARCSYNC_RESET_DEASSERT)
		pwd = (arcsync->version >= 2) ? ARCSYNC2_RESET_DEASSERT : ARCSYNC1_RESET_PWD;
	else
		pwd = (arcsync->version >= 2) ? ARCSYNC2_RESET_ASSERT : ARCSYNC1_RESET_PWD;

	mutex_lock(&arcsync->lock);
	writel(coreid + pwd, arcsync->regs + reg_offset);
	mutex_unlock(&arcsync->lock);

	return 0;
}

/**
 * arcsync_start() - send a start signal
 * @clid: cluster ID
 * @cid: core number inside cluster
 *
 * Send run request to the specified core
 *
 * Return: 0 on success or negative errno on failure.
 */
static int arcsync_start(struct device *dev, u32 clid, u32 cid)
{
	u32 count = 10;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reg_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_RUN(coreid) :
						   ARCSYNC1_CORE_RUN(coreid);

	mutex_lock(&arcsync->lock);

	/* Ensure that start handshake is no running */
	while (readl(arcsync->regs + reg_offset) && --count)
		udelay(1);

	if (count)
		writel(1, arcsync->regs + reg_offset);

	mutex_unlock(&arcsync->lock);

	return count ? 0 : -EBUSY;
}

/**
 * arcsync_halt() - send a halt signal
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @cid: core number inside cluster
 *
 * Send halt request to the specified core
 *
 * Return: 0 on success or negative errno on failure.
 */
static int arcsync_halt(struct device *dev, u32 clid, u32 cid)
{
	u32 count = 10;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reg_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_HALT(coreid) :
						   ARCSYNC1_CORE_HALT(coreid);

	mutex_lock(&arcsync->lock);

	/* Ensure halt handshake is no running */
	while (readl(arcsync->regs + reg_offset) && --count)
		udelay(1);

	if (count)
		writel(1, arcsync->regs + reg_offset);

	mutex_unlock(&arcsync->lock);

	return count ? 0 : -EBUSY;
}

/**
 * arcsync_set_ivt() - set the interrupt vector table base address for the core
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @cid: core number inside cluster
 * @ivt_addr: interrupt vector table address
 *
 * Set IVT for the specified core. The driver reads IVT add from the
 * firmware elf .vector section.
 *
 * Return: 0
 */
static int
arcsync_set_ivt(struct device *dev, u32 clid, u32 cid, phys_addr_t ivt_addr)
{
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reglo_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_IVT_LO(coreid) :
						     ARCSYNC1_CORE_IVT_LO(coreid);
	u32 reghi_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_IVT_HI(coreid) :
						     ARCSYNC1_CORE_IVT_HI(coreid);
	u32 shift_ivt = 10;

	dev_dbg(arcsync->dev, "ARCsync set IVT to %pa on Core %d\n",
		&ivt_addr, coreid);

	mutex_lock(&arcsync->lock);

	if (arcsync->vdk_fix)
		shift_ivt = 0;

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	writel(ivt_addr >> shift_ivt, arcsync->regs + reglo_offset);
	writel(ivt_addr >> 32, arcsync->regs + reghi_offset);
#else
	writel(ivt_addr >> shift_ivt, arcsync->regs + reglo_offset);
	writel(0, arcsync->regs + reghi_offset);
#endif
	mutex_unlock(&arcsync->lock);

	return 0;
}

/**
 * arcsync_get_status() - get status of the specified core
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @cid: core number inside cluster
 *
 * Read and return the core running status.
 *
 * Return: core status
 */
static int arcsync_get_status(struct device *dev, u32 clid, u32 cid)
{
	u32 status;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);
	u32 coreid = arcsync_build_coreid(clid, cid, arcsync->corenum_width);
	u32 reg_offset = (arcsync->version >= 2) ? ARCSYNC2_CORE_STATUS(coreid) :
						   ARCSYNC1_CORE_STATUS(coreid);
	int ret_status = 0;

	mutex_lock(&arcsync->lock);
	status = readl(arcsync->regs + reg_offset);
	mutex_unlock(&arcsync->lock);

	if (status & ARCSYNC_CORE_STATUS_HALT)
		ret_status |= ARCSYNC_CORE_HALTED;

	if ((status & ARCSYNC_CORE_STATUS_PWDOWN_M1) ||
	    (status & ARCSYNC_CORE_STATUS_PWDOWN_M2))
		ret_status |= ARCSYNC_CORE_POWERDOWN;

	if (status & ARCSYNC_CORE_STATUS_SLEEP)
		ret_status |= ARCSYNC_CORE_SLEEPING;

	if (!ret_status)
		ret_status = ARCSYNC_CORE_RUNNING;

	return ret_status;
}

/**
 * arcsync_reset_cluster_group() - reset the NPX L2 group or L1 slice group
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @grp: group ID
 * @cmd: reset command ARCSYNC_RESET_DEASSERT or ARCSYNC_RESET_ASSERT
 *
 * Assert or de-assert reset line for the group of slices
 *
 * Return: 0 on success or negative errno on failure.
 */
static int
arcsync_reset_cluster_group(struct device *dev, u32 clid, u32 grp, u32 cmd)
{
	uint32_t shift_by;
	uint32_t val;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	if (arcsync->version == 1)
		return 0;

	if (grp > ARCSYNC_NPX_L2GRP)
		return 0;

	if (grp == 4)
		shift_by = 0;
	else if (grp == 3)
		shift_by = 12;
	else if (grp == 2)
		shift_by = 9;
	else if (grp == 1)
		shift_by = 6;
	else
		shift_by = 3;

	if (cmd == ARCSYNC_RESET_DEASSERT)
		val = ((grp + 1) << shift_by) + ARCSYNC2_RESET_DEASSERT;
	else
		val = ((grp + 1) << shift_by) + ARCSYNC2_RESET_ASSERT;

	mutex_lock(&arcsync->lock);
	writel(val, arcsync->regs + ARCSYNC2_CL_GRP_RST(clid));
	mutex_unlock(&arcsync->lock);

	return 0;
}

/**
 * arcsync_clk_ctrl_cluster_group() - enable/disable the group clock
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @grp: group ID
 * @cmd: clock control command ARCSYNC_CLK_DIS or ARCSYNC_CLK_EN
 *
 * Controlling the slice group clock enable/disable
 *
 * Return: 0
 */
static int
arcsync_clk_ctrl_cluster_group(struct device *dev, u32 clid, u32 grp, u32 cmd)
{
	u32 shift_by;
	u32 val;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	if (arcsync->version == 1)
		return 0;

	if (grp > ARCSYNC_NPX_L2GRP)
		return 0;

	if (grp == 4) {
		/* Nothing to do for L2 group */
		return 0;
	} else if (grp == 3) {
		shift_by = 12;
	} else if (grp == 2) {
		shift_by = 9;
	} else if (grp == 1) {
		shift_by = 6;
	} else {
		shift_by = 3;
	}
	if (cmd == ARCSYNC_CLK_DIS)
		val = ((grp + 1) << shift_by) + ARCSYNC2_GRP_CLK_PWD_DIS;
	else
		val = ((grp + 1) << shift_by) + ARCSYNC2_GRP_CLK_PWD_EN;

	mutex_lock(&arcsync->lock);
	writel(val, arcsync->regs + ARCSYNC2_CL_GRP_CLK_EN(clid));
	mutex_unlock(&arcsync->lock);

	return 0;
}

/**
 * arcsync_power_ctrl_cluster_group() - group power control
 * @dev: arcsync device handle
 * @clid: cluster ID
 * @grp: group ID
 * @cmd: power control command ARCSYNC_POWER_UP or ARCSYNC_POWER_DOWN
 *
 * Set group power up or power down state.
 *
 * Return: 0
 */
static int
arcsync_power_ctrl_cluster_group(struct device *dev, u32 clid, u32 grp, u32 cmd)
{
	u32 offset = 0;
	u32 val;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	if (arcsync->version == 1)
		return 0;

	if (cmd == ARCSYNC_POWER_UP)
		val = ARCSYNC2_GRP_POWERUP;
	else
		val = ARCSYNC2_GRP_POWERDOWN;

	switch (grp) {
	case ARCSYNC_NPX_L1GRP0:
		offset = ARCSYNC2_CL_GRP0_PMOD(clid);
		break;
	case ARCSYNC_NPX_L1GRP1:
		offset = ARCSYNC2_CL_GRP1_PMOD(clid);
		break;
	case ARCSYNC_NPX_L1GRP2:
		offset = ARCSYNC2_CL_GRP2_PMOD(clid);
		break;
	case ARCSYNC_NPX_L1GRP3:
		offset = ARCSYNC2_CL_GRP3_PMOD(clid);
		break;
	}

	if (offset) {
		mutex_lock(&arcsync->lock);
		writel(val, arcsync->regs + offset);
		mutex_unlock(&arcsync->lock);
	}

	return 0;
}

static struct arcsync_interrupt *
arcsync_get_interrupt(struct arcsync_device *arcsync, u32 irq)
{
	int i;

	for (i = 0; i < arcsync->num_irqs; i++) {
		if (arcsync->irq[i].irqnum == irq)
			return &arcsync->irq[i];
	}

	return NULL;
}

/**
 * arcsync_set_interrupt_callback() - add callback for ARCSync interrupt handler
 * @dev: arcsync device handle
 * @irq: irq num
 * @func: callback function pointer
 * @data: data pointer
 *
 * Add callback to an interrupt callback list. If the external driver needs
 * some action for ARCSync IRQ in registers callback.
 *
 * Return: 0 on success or negative errno on failure.
 */
static int
arcsync_set_interrupt_callback(struct device *dev, u32 irq,
			       intr_callback_t func, void *data)
{
	struct arcsync_callback *cb;
	struct arcsync_interrupt *intr;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	intr = arcsync_get_interrupt(arcsync, irq);
	if (intr == NULL)
		return -EINVAL;

	cb = kmalloc(sizeof(struct arcsync_callback), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	cb->func = func;
	cb->data = data;

	spin_lock_irq(&intr->callbacks_list_lock);
	list_add(&cb->link, &intr->callbacks_list);
	spin_unlock_irq(&intr->callbacks_list_lock);

	return 0;
}

/**
 * arcsync_remove_interrupt_callback() - remove interrupt handler callback
 * @dev: arcsync device handle
 * @irq: irq num
 * @data: data pointer
 *
 * Remove the callback from an interrupt callback list.
 *
 * Return: 0 on success or negative errno on failure.
 */
static int
arcsync_remove_interrupt_callback(struct device *dev, u32 irq,
				  void *data)
{
	struct arcsync_interrupt *intr;
	struct arcsync_callback *cb, *tmp;
	struct arcsync_callback *remove_cb = NULL;
	struct arcsync_device *arcsync = dev_get_drvdata(dev);

	intr = arcsync_get_interrupt(arcsync, irq);
	if (intr == NULL)
		return -EINVAL;

	spin_lock_irq(&intr->callbacks_list_lock);
	list_for_each_entry_safe(cb, tmp, &intr->callbacks_list, link) {
		if (cb->data == data) {
			list_del(&cb->link);
			remove_cb = cb;
			break;
		}
	}
	spin_unlock_irq(&intr->callbacks_list_lock);

	kfree(remove_cb);

	return 0;
}

static const struct arcsync_funcs arcsync_ctrl = {
	.get_version = arcsync_version,
	.get_has_pmu = arcsync_has_pmu,
	.get_arcnet_id = arcsync_arcnet_id,
	.get_virt_mode = arcsync_virt_mode,
	.get_host_cluster_id = arcsync_host_cluster_id,
	.build_coreid = arcsync_build_coreid_fn,
	.clk_ctrl = arcsync_clk_ctrl,
	.power_ctrl = arcsync_power_ctrl,
	.reset = arcsync_reset,
	.start = arcsync_start,
	.halt = arcsync_halt,
	.set_ivt = arcsync_set_ivt,
	.get_status = arcsync_get_status,
	.reset_cluster_group = arcsync_reset_cluster_group,
	.clk_ctrl_cluster_group = arcsync_clk_ctrl_cluster_group,
	.power_ctrl_cluster_group = arcsync_power_ctrl_cluster_group,
	.set_interrupt_callback = arcsync_set_interrupt_callback,
	.remove_interrupt_callback = arcsync_remove_interrupt_callback,
};

static int arcsync_get_clusters_num(struct arcsync_device *arcsync)
{
	u32 bcr;

	bcr = readl(arcsync->regs + ARCSYNC_BLD_CFG);
	return ARCSYNC_BLD_CUSTERS_NUM(bcr);
}

static int arcsync_get_cores_per_cluster(struct arcsync_device *arcsync)
{
	u32 bcr;
	u32 cores;

	bcr = readl(arcsync->regs + ARCSYNC_BLD_CFG);
	cores = ARCSYNC_BLD_CORES_PER_CL(bcr);

	if (cores > 32) {
		dev_dbg(arcsync->dev,
			"Warning cores per cluster %d, set to 32\n", cores);
		cores = 32;
	}

	return cores;
}

static int arcsync_get_ac_num(struct arcsync_device *arcsync)
{
	u32 bcr;

	bcr = readl(arcsync->regs + ARCSYNC_BLD_CFG);
	return ARCSYNC_BLD_AC_NUM(bcr);
}

static int arcsync_get_vm_num(struct arcsync_device *arcsync)
{
	u32 bcr;

	bcr = readl(arcsync->regs + ARCSYNC_BLD_CFG);
	return ARCSYNC_BLD_NUM_VMS(bcr);
}

static void arcsync_setup_default_vm_map(struct arcsync_device *arcsync)
{
	u32 vm, vp;
	u32 num_vp = arcsync->clusters_num;
	u32 num_vm = arcsync->vm_num;

	if (!num_vm || arcsync->version < 2) {
		dev_warn(arcsync->dev, "ARCSync does not support VMs\n");
		return;
	}

	mutex_lock(&arcsync->lock);
	for (vm = 0; vm < num_vm; vm++) {
		for (vp = 0; vp < num_vp; vp++) {
			u32 off = ARCSYNC2_VM_MAP_OFFSET + 4 * (vm * num_vp + vp);
			writel(ARCSYNC2_VM_MAP_VALID | vp, arcsync->regs + off);
		}
	}
	mutex_unlock(&arcsync->lock);
}

static int arcsync_read_version(struct arcsync_device *arcsync)
{
	return readl(arcsync->regs + ARCSYNC_BLD_CFG) & ARCSYNC_BLD_VERSION_MASK;
}

static int arcsync_read_has_pmu(struct arcsync_device *arcsync)
{
	return (readl(arcsync->regs + ARCSYNC_BLD_CFG) & ARCSYNC_BLD_HAS_PMU) ? 1 : 0;
}

/**
 * arcsync_get_device_by_phandle() - find an ARCSync device handle by phandle
 * @np - caller driver device node pointer
 * @phandle_name - string with property name containing phandle
 *
 * Look up and return the ARCSync device handle corresponding to the
 * @phanlde_name. If no device can be found, this returns error code.
 *
 * Return: pointer to the device handle or negative errno on failure.
 */
struct device *arcsync_get_device_by_phandle(struct device_node *np,
					     const char *phandle_name)
{
	struct platform_device *pdev;
	struct device_node *arcsync_np;

	arcsync_np = of_parse_phandle(np, phandle_name, 0);
	if (!arcsync_np)
		return ERR_PTR(-EINVAL);

	if (!of_match_node(snps_arcsync_platform_driver.driver.of_match_table,
	    arcsync_np)) {
		of_node_put(arcsync_np);
		return ERR_PTR(-EINVAL);
	}

	pdev = of_find_device_by_node(arcsync_np);
	of_node_put(arcsync_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	return &pdev->dev;
}
EXPORT_SYMBOL(arcsync_get_device_by_phandle);

/**
 * arcsync_get_ctrl_fn - get struct with ARCSync control functions
 * @dev: arcsync device handle
 *
 * Returns pointer to the structure with ARCSync functions provided be the
 * driver.
 *
 * Return: pointer or negative errno on failure.
 */
const struct arcsync_funcs *arcsync_get_ctrl_fn(struct device *dev)
{
	struct arcsync_device *arcsync;

	if (!dev)
		return ERR_PTR(-EINVAL);

	arcsync = dev_get_drvdata(dev);
	if (!arcsync)
		return ERR_PTR(-EINVAL);

	return arcsync->funcs;
}
EXPORT_SYMBOL(arcsync_get_ctrl_fn);

/**
 * arcsync_interrupt - arcsync interrupt handler
 * @irq: IRQ number
 * @idata: Pointer to the arcsync interrupt structure
 */
static irqreturn_t arcsync_interrupt(int irq, void *idata)
{
	struct arcsync_interrupt *irq_data = (struct arcsync_interrupt *)idata;
	struct arcsync_device *arcsync = irq_data->arcsync;
	struct arcsync_callback *cb;
	u32 offs;
	u32 val;

	if (irq != irq_data->irqnum) {
		dev_dbg(arcsync->dev, "Received IRQ %d (idx %u), expected %u\n",
			irq, irq_data->idx, irq_data->irqnum);
		return IRQ_NONE;
	}

	offs = irq_data->ack_offs;
	val = readl(arcsync->regs + offs);
	if (!val)
		return IRQ_HANDLED;

	writel(arcsync->host_id, arcsync->regs + offs);

	/* Use this interrupt as a doorbell for application drivers. We can't
	 * determine what firmware app generated an IRQ,
	 * call every callback, to unblock user space apps, they will figure
	 * out what to do.
	 * We don't share callbacks_list_lock between different interrupt
	 * handlers, each handler has its own lock, so we can use a simple
	 * spin_lock.
	 */
	spin_lock(&irq_data->callbacks_list_lock);
	list_for_each_entry(cb, &irq_data->callbacks_list, link) {
		if (cb && cb->func)
			cb->func(irq, cb->data);
	}
	spin_unlock(&irq_data->callbacks_list_lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_ISA_ARCV2
/* Read ARC host CPU cluster ID and core ID and build ARCSync core ID */
static u32 arc_read_host_coreid(void)
{
	return (read_aux_reg(ARC_AUX_IDENTITY) >> 8) & 0xFF;
}

static u32 arc_read_host_clusterid(void)
{
	return read_aux_reg(ARC_AUX_CLUSTER_ID) & 0xFF;
}
#endif

static int arcsync_probe(struct platform_device *pdev)
{
	const struct arcsync_dev_data *arcsync_data;
	struct arcsync_device *arcsync;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	u32 cores_per_cluster;
	u32 hcluster_id = 0;
	u32 hcore_id = 0;
	u32 irqs_per_vm = 1;
	int ret;
	int i;

	arcsync = devm_kzalloc(&pdev->dev, sizeof(*arcsync), GFP_KERNEL);
	if (!arcsync)
		return -ENOMEM;

	arcsync->dev = &pdev->dev;

	arcsync_data = of_device_get_match_data(&pdev->dev);
	if (arcsync_data && arcsync_data->virt_irq)
		arcsync->virt_irq = true;

	ret = platform_irq_count(pdev);
	if (!ret) {
		dev_warn(&pdev->dev, "No IRQ specified, continue without IRQ handler\n");
	} else if (ret < 0) {
		dev_err(&pdev->dev, "Failed to count IRQs: %d\n", ret);
		return ret;
	}

	if (ret <= ARCSYNC_HOST_MAX_IRQS) {
		arcsync->num_irqs = ret;
	} else {
		dev_warn(&pdev->dev,
			 "Specified more IRQs than supported, continue with first %d IRQs\n",
			 ARCSYNC_HOST_MAX_IRQS);
		arcsync->num_irqs = ARCSYNC_HOST_MAX_IRQS;
	}

	/*
	 * In virtual IRQ mode, map the linear DT interrupt index to a
	 * (VM ID, IRQ index) pair. IRQs are grouped by VM, with the number
	 * of IRQs per VM defined by "snps,irqs-per-vm" (default 1, the
	 * one-IRQ-per-VM layout).
	 */
	if (arcsync->virt_irq) {
		of_property_read_u32(node, "snps,irqs-per-vm", &irqs_per_vm);
		if (irqs_per_vm == 0)
			irqs_per_vm = 1;
		if (arcsync->num_irqs % irqs_per_vm)
			dev_warn(&pdev->dev,
				 "IRQ count %u is not a multiple of irqs-per-vm %u\n",
				 arcsync->num_irqs, irqs_per_vm);
	}

	for (i = 0; i < arcsync->num_irqs; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(&pdev->dev, "Could not get irq[%d]\n", i);
			return ret;
		}
		arcsync->irq[i].irqnum = ret;
		arcsync->irq[i].idx = i;
		arcsync->irq[i].vm_id = i / irqs_per_vm;
		arcsync->irq[i].vm_irq_idx = i % irqs_per_vm;
		arcsync->irq[i].arcsync = arcsync;
		spin_lock_init(&arcsync->irq[i].callbacks_list_lock);
		INIT_LIST_HEAD(&arcsync->irq[i].callbacks_list);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	arcsync->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(arcsync->regs)) {
		dev_err(&pdev->dev, "Could not map ARCsync registers\n");
		return PTR_ERR(arcsync->regs);
	}

	if (of_property_read_bool(node, "snps,vdk-fix"))
		arcsync->vdk_fix = 1;

	mutex_init(&arcsync->lock);
	arcsync->funcs = &arcsync_ctrl;
	arcsync->has_pmu = arcsync_read_has_pmu(arcsync);
	arcsync->version = arcsync_read_version(arcsync);
	if (arcsync->version < 2) {
		arcsync->virt_irq = false;
		dev_warn(&pdev->dev, "ARCSync v1 does not support virtualization\n");
	}

	arcsync->clusters_num = arcsync_get_clusters_num(arcsync);
	if (arcsync->vdk_fix)
		arcsync->clusters_num -= 1;
	cores_per_cluster = arcsync_get_cores_per_cluster(arcsync);
	arcsync->cores_max = arcsync->clusters_num * cores_per_cluster;
	if (arcsync->version > 1)
		arcsync->ac_num = arcsync_get_ac_num(arcsync);

	if (arcsync->version >= 2)
		arcsync->vm_num = arcsync_get_vm_num(arcsync);

	arcsync->corenum_width = ilog2(cores_per_cluster);

	if (!of_property_read_u32(node, "snps,host-cluster-id", &hcluster_id)) {
		of_property_read_u32(node, "snps,host-core-id", &hcore_id);
	} else {
#ifdef CONFIG_ISA_ARCV2
		hcluster_id = arc_read_host_clusterid();
		hcore_id = arc_read_host_coreid();
#else
		hcluster_id = ARCSYNC_DEF_HOST_CLUSTER_ID;
#endif
	}
	if (arcsync->virt_irq)
		arcsync->host_id = hcluster_id;
	else
		arcsync->host_id = arcsync_build_coreid(hcluster_id, hcore_id,
							arcsync->corenum_width);

	of_property_read_u32(node, "snps,arcnet-id", &arcsync->arcnet_id);

	if (of_property_read_bool(node, "snps,default-vm-map"))
		arcsync_setup_default_vm_map(arcsync);

	dev_dbg(&pdev->dev, "ARCsync registers addr %pap (mapped %pS)\n",
		&res->start, arcsync->regs);

	dev_dbg(&pdev->dev, "ARCnet id 0x%x\n", arcsync->arcnet_id);
	dev_dbg(&pdev->dev, "Clusters num: %d\n", arcsync->clusters_num);
	dev_dbg(&pdev->dev, "Cores num: %d\n", arcsync->cores_max);
	dev_dbg(&pdev->dev, "Corenum width: %d\n", arcsync->corenum_width);
	dev_dbg(&pdev->dev, "Atomic counters: %d\n", arcsync->ac_num);
	dev_dbg(&pdev->dev, "VMs num: %d\n", arcsync->vm_num);
	dev_dbg(&pdev->dev, "PMU: %d\n", arcsync->has_pmu);
	dev_dbg(&pdev->dev, "VDK fix: %d\n", arcsync->vdk_fix);
	dev_dbg(&pdev->dev, "Host ID 0x%x\n", arcsync->host_id);

	platform_set_drvdata(pdev, arcsync);

	for (i = 0; i < arcsync->num_irqs; i++) {
		const char *name = NULL;

		if (arcsync->virt_irq)
			arcsync->irq[i].ack_offs = ARCSYNC2_VM_EID_ACK_IRQ(
							arcsync->irq[i].vm_id,
							arcsync->irq[i].vm_irq_idx,
							arcsync->host_id);
		else
			arcsync->irq[i].ack_offs = ARCSYNC2_EID_ACK_IRQ(
							arcsync->host_id,
							arcsync->irq[i].idx);

		dev_dbg(&pdev->dev, "Request IRQ: %d\n", arcsync->irq[i].irqnum);

		of_property_read_string_index(node, "interrupt-names", i, &name);
		if (name)
			snprintf(arcsync->irq[i].name, sizeof(arcsync->irq[i].name),
				 "arcsync%d-%s", arcsync->arcnet_id, name);
		else
			snprintf(arcsync->irq[i].name, sizeof(arcsync->irq[i].name),
				 "arcsync%d-irq%d", arcsync->arcnet_id, i);

		ret = devm_request_irq(arcsync->dev, arcsync->irq[i].irqnum,
				       arcsync_interrupt,
				       0,
				       arcsync->irq[i].name,
				       &arcsync->irq[i]);
		if (ret) {
			dev_err(&pdev->dev, "Failed to set interrupt handler for %d IRQ\n",
				arcsync->irq[i].irqnum);
			return ret;
		}
	}

	return ret;
}

#ifdef CONFIG_OF
static const struct arcsync_dev_data arcsync_phys_data = {
	.virt_irq = false,
};

static const struct arcsync_dev_data arcsync_virt_data = {
	.virt_irq = true,
};

static const struct of_device_id snps_arcsync_match[] = {
	{ .compatible = "snps,arcsync-virt", .data = &arcsync_virt_data },
	{ .compatible = "snps,arcsync", .data = &arcsync_phys_data },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, snps_arcsync_match);
#endif

static struct platform_driver snps_arcsync_platform_driver = {
	.probe = arcsync_probe,
	.driver = {
		.name = "arcsync",
		.of_match_table = of_match_ptr(snps_arcsync_match),
	},
};

module_platform_driver(snps_arcsync_platform_driver);

MODULE_AUTHOR("Synopsys Inc.");
MODULE_DESCRIPTION("ARCsync driver");
MODULE_LICENSE("GPL v2");
