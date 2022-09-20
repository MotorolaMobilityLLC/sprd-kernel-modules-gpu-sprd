/*
 *
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */



#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/hwspinlock.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include <mali_kbase_debug.h>
//For fix r24p0 --> r27p0 compile error.
//#include <backend/gpu/mali_kbase_device_internal.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#ifdef KBASE_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/regulator/consumer.h>

#include <linux/smp.h>
#include <trace/events/power.h>
#include <linux/soc/sprd/hwfeature.h>
#include <linux/nvmem-consumer.h>
#include <linux/thermal.h>
#include <gpu,qogirn6l-regs.h>
#include <gpu,qogirn6l-mask.h>
#include "mali_kbase_gpu_sys_qogirn6l_qos.h"


#define MOVE_BIT_LEFT(x,n) ((unsigned long)x << n)

#define VENDOR_FTRACE_MODULE_NAME    "unisoc-gpu"

#define DTS_CLK_OFFSET      2
#define PM_RUNTIME_DELAY_MS 50
#define UP_THRESHOLD        9/10
#define FREQ_KHZ            1000
#define VOLT_KMV            1000
#define GPU_MIN_FREQ_INDEX  3

#define GPU_26M_FREQ        26000000
#define GPU_76M8_FREQ       76800000
#define GPU_153M6_FREQ      153600000

#define T770_GPLL_780M      780000000
#define T770_GPLL_686M      686400000
#define T760_GPLL_655M      655200000

#define GPU_680M_FREQ       680000000
#define GPU_850M_FREQ       850000000

#define VOLT_650mV 650000
#define VOLT_700mV 700000
#define VOLT_750mV 750000
#define VOLT_800mV 800000
unsigned long gpll_T770_1950M, gpll_T770_1716M, gpll_T760_1638M;

#if 0
struct gpu_qos_config {
	u8 arqos;
	u8 awqos;
	u8 arqos_threshold;
	u8 awqos_threshold;
};
#endif

struct gpu_freq_info {
	struct clk* clk_src;
	int freq;	//kHz
	int volt;	//uV
	//int dvfs_index;
};

struct gpu_reg_info {
	struct regmap* regmap_ptr;
	uint32_t args[2];
};

struct gpu_dvfs_context {
	int gpu_clock_on;
	int gpu_power_on;
	int cur_voltage;

	struct clk*  clk_gpu_i;
	struct clk*  clk_gpu_core_eb;
	struct clk** gpu_clk_src;
	int gpu_clk_num;

	struct gpu_freq_info* freq_list;
	int freq_list_len;

	int cur_index;
	const struct gpu_freq_info* freq_cur;
	const struct gpu_freq_info* freq_default;

	struct semaphore* sem;

	struct gpu_reg_info top_dvfs_cfg_reg;
	struct gpu_reg_info gpu_sw_dvfs_ctrl_reg;
	struct gpu_reg_info top_force_reg;
	struct gpu_reg_info gpu_top_state_reg;
	struct gpu_reg_info gpll_cfg_force_off_reg;
	struct gpu_reg_info gpll_cfg_force_on_reg;
	struct gpu_reg_info dcdc_gpu_voltage0;
	struct gpu_reg_info dcdc_gpu_voltage1;
	struct gpu_reg_info dcdc_gpu_voltage2;
	struct gpu_reg_info dcdc_gpu_voltage3;
#if 0
	struct gpu_reg_info gpu_qos_sel;
	struct gpu_reg_info gpu_qos;
#endif
	struct gpu_reg_info dvfs_index_cfg;
	struct gpu_reg_info sw_dvfs_ctrl;
	struct gpu_reg_info freq_upd_cfg;
	struct gpu_reg_info core_index0_map;
	struct gpu_reg_info core_index1_map;
	struct gpu_reg_info core_index2_map;
	struct gpu_reg_info core_index3_map;
	struct gpu_reg_info core_index4_map;
	struct gpu_reg_info core_index5_map;
	struct gpu_reg_info core_index6_map;
	struct gpu_reg_info core_index6_map_sel;
	struct gpu_reg_info core_index7_map;

	struct regmap* gpu_apb_base_ptr;
	struct regmap* gpu_dvfs_apb_base_ptr;

	struct reset_control* gpu_soft_rst;

	struct regulator *gpu_reg_ptr;

	const char* auto_efuse;
	u32 gpu_binning;

	int last_gpu_temperature;
	int gpu_temperature;
};

DEFINE_SEMAPHORE(gpu_dfs_sem);
static struct gpu_dvfs_context gpu_dvfs_ctx=
{
	.gpu_clock_on = 0,
	.gpu_power_on = 0,
	.last_gpu_temperature = 0,
	.gpu_temperature = 0,
	.sem = &gpu_dfs_sem,
};

#if 0
static struct gpu_qos_config gpu_qos_cfg=
{
	.arqos = 0,
	.awqos = 0,
	.arqos_threshold = 0,
	.awqos_threshold = 0,
};
#endif

int gpu_boost_level;
#if 0
void __iomem *mali_qos_reg_base_mtx_m0;
void __iomem *mali_qos_reg_base_mtx_m1;
void __iomem *mali_qos_reg_base_apb_rf;
#ifdef CONFIG_MALI_DEVFREQ
static void InitFreqStats(struct kbase_device *kbdev)
{
	int i = 0;

	kbdev->enable_freq_stats = 0;
	kbdev->freq_num = gpu_dvfs_ctx.freq_list_len;
	kbdev->freq_stats = vmalloc(sizeof(struct kbase_devfreq_stats) * kbdev->freq_num);
	KBASE_DEBUG_ASSERT(kbdev->freq_stats);

	for (i = 0; i < kbdev->freq_num; i++)
	{
		kbdev->freq_stats[i].freq = gpu_dvfs_ctx.freq_list[i].freq * FREQ_KHZ;
		kbdev->freq_stats[i].busy_time = 0;
		kbdev->freq_stats[i].total_time = 0;
	}
}

static void DeinitFreqStats(struct kbase_device *kbdev)
{
	if (NULL != kbdev->freq_stats)
	{
		vfree(kbdev->freq_stats);
		kbdev->freq_stats = NULL;
	}
}

#endif
#endif

#if 0
static int sprd_gpu_cal_read(struct device_node *np, const char *cell_id, u32 *val)
{
	struct nvmem_cell *cell;
	void *buf;
	size_t len;

	cell = of_nvmem_cell_get(np, cell_id);
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf))
	{
		nvmem_cell_put(cell);
		return PTR_ERR(buf);
	}

	memcpy(val, buf, min(len, sizeof(u32)));

	kfree(buf);
	nvmem_cell_put(cell);

	return 0;
}
#endif

#if 0
static inline void mali_freq_init(struct device *dev)
{
	int i = 0, clk_cnt = 0, ret = 0;
	//struct device_node *qos_node = NULL;
	struct device_node *hwf;

	gpu_dvfs_ctx.gpu_reg_ptr = devm_regulator_get(dev, "gpu");
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_reg_ptr);

	gpu_dvfs_ctx.gpu_soft_rst = devm_reset_control_get(dev, "gpu_soft_rst");
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_soft_rst);

	gpu_dvfs_ctx.top_dvfs_cfg_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"top_dvfs_cfg", 2, (uint32_t *)gpu_dvfs_ctx.top_dvfs_cfg_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.top_dvfs_cfg_reg.regmap_ptr);

	gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"gpu_sw_dvfs_ctrl", 2, (uint32_t *)gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.regmap_ptr);

	gpu_dvfs_ctx.dcdc_gpu_voltage0.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"dcdc_gpu_voltage0", 2, (uint32_t *)gpu_dvfs_ctx.dcdc_gpu_voltage0.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.dcdc_gpu_voltage0.regmap_ptr);

	gpu_dvfs_ctx.dcdc_gpu_voltage1.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"dcdc_gpu_voltage1", 2, (uint32_t *)gpu_dvfs_ctx.dcdc_gpu_voltage1.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.dcdc_gpu_voltage1.regmap_ptr);

	gpu_dvfs_ctx.dcdc_gpu_voltage2.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"dcdc_gpu_voltage2", 2, (uint32_t *)gpu_dvfs_ctx.dcdc_gpu_voltage2.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.dcdc_gpu_voltage2.regmap_ptr);

	gpu_dvfs_ctx.dcdc_gpu_voltage3.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"dcdc_gpu_voltage3", 2, (uint32_t *)gpu_dvfs_ctx.dcdc_gpu_voltage3.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.dcdc_gpu_voltage3.regmap_ptr);

	//enable gpu hw dvfs
	regmap_update_bits(gpu_dvfs_ctx.top_dvfs_cfg_reg.regmap_ptr, gpu_dvfs_ctx.top_dvfs_cfg_reg.args[0], gpu_dvfs_ctx.top_dvfs_cfg_reg.args[1], ~gpu_dvfs_ctx.top_dvfs_cfg_reg.args[1]);
	regmap_update_bits(gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.regmap_ptr, gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.args[0], gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.args[1], ~gpu_dvfs_ctx.gpu_sw_dvfs_ctrl_reg.args[1]);

	gpu_dvfs_ctx.top_force_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"top_force_shutdown", 2, (uint32_t *)gpu_dvfs_ctx.top_force_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.top_force_reg.regmap_ptr);

	gpu_dvfs_ctx.gpu_top_state_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"gpu_top_state", 2, (uint32_t *)gpu_dvfs_ctx.gpu_top_state_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_top_state_reg.regmap_ptr);

	gpu_dvfs_ctx.gpll_cfg_force_off_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"gpll_cfg_frc_off", 2, (uint32_t *)gpu_dvfs_ctx.gpll_cfg_force_off_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpll_cfg_force_off_reg.regmap_ptr);

	gpu_dvfs_ctx.gpll_cfg_force_on_reg.regmap_ptr = syscon_regmap_lookup_by_phandle_args(dev->of_node,"gpll_cfg_frc_on", 2, (uint32_t *)gpu_dvfs_ctx.gpll_cfg_force_on_reg.args);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpll_cfg_force_on_reg.regmap_ptr);

	gpu_dvfs_ctx.gpu_apb_base_ptr = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,gpu-apb-syscon");
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_apb_base_ptr);
	gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,gpu-dvfs-apb-syscon");
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr);

#if 0
	//qos
	gpu_dvfs_ctx.gpu_qos_sel.regmap_ptr = gpu_dvfs_ctx.gpu_apb_base_ptr;
	gpu_dvfs_ctx.gpu_qos_sel.args[0] = REG_GPU_APB_RF_GPU_NIC400_QOS;
	gpu_dvfs_ctx.gpu_qos_sel.args[1] = MASK_GPU_APB_RF_GPU_QOS_SEL;

	gpu_dvfs_ctx.gpu_qos.regmap_ptr = gpu_dvfs_ctx.gpu_apb_base_ptr;
	gpu_dvfs_ctx.gpu_qos.args[0] = REG_GPU_APB_RF_GPU_NIC400_QOS;
	gpu_dvfs_ctx.gpu_qos.args[1] = MASK_GPU_APB_RF_AWQOS_THRESHOLD_GPU | MASK_GPU_APB_RF_ARQOS_THRESHOLD_GPU | MASK_GPU_APB_RF_AWQOS_GPU | MASK_GPU_APB_RF_ARQOS_GPU;

	/* qos dts parse */
	qos_node = of_parse_phandle(dev->of_node, "sprd,qos", 0);
	if (qos_node)
	{
		if (of_property_read_u8(qos_node, "arqos", &gpu_qos_cfg.arqos)) {
			pr_warn("gpu arqos config reading fail.\n");
		}
		if (of_property_read_u8(qos_node, "awqos", &gpu_qos_cfg.awqos)) {
			pr_warn("gpu awqos config reading fail.\n");
		}
		if (of_property_read_u8(qos_node, "arqos-threshold", &gpu_qos_cfg.arqos_threshold)) {
			pr_warn("gpu arqos_threshold config reading fail.\n");
		}
		if (of_property_read_u8(qos_node, "awqos-threshold", &gpu_qos_cfg.awqos_threshold)) {
			pr_warn("gpu awqos_threshold config reading fail.\n");
		}
	} else {
		pr_warn("can't find gpu qos config node\n");
	}
#endif

	//gpu index cfg
	gpu_dvfs_ctx.dvfs_index_cfg.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.dvfs_index_cfg.args[0] = REG_GPU_DVFS_APB_RF_GPU_DVFS_INDEX_CFG;
	gpu_dvfs_ctx.dvfs_index_cfg.args[1] = MASK_GPU_DVFS_APB_RF_GPU_DVFS_INDEX;

	//sw dvfs ctrl
	gpu_dvfs_ctx.sw_dvfs_ctrl.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.sw_dvfs_ctrl.args[0] = REG_GPU_DVFS_APB_RF_GPU_SW_DVFS_CTRL;
	gpu_dvfs_ctx.sw_dvfs_ctrl.args[1] = MASK_GPU_DVFS_APB_RF_GPU_DVFS_ACK | MASK_GPU_DVFS_APB_RF_GPU_DVFS_VOLTAGE_SW | MASK_GPU_DVFS_APB_RF_GPU_DVFS_REQ_SW;

	//freq update cfg
	gpu_dvfs_ctx.freq_upd_cfg.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.freq_upd_cfg.args[0] = REG_GPU_DVFS_APB_RF_GPU_FREQ_UPD_TYPE_CFG0;
	gpu_dvfs_ctx.freq_upd_cfg.args[1] = MASK_GPU_DVFS_APB_RF_GPU_FREQ_UPD_HDSK_EN | MASK_GPU_DVFS_APB_RF_GPU_FREQ_UPD_DELAY_EN;

	//core index0 map
	gpu_dvfs_ctx.core_index0_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index0_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX0_MAP;
	gpu_dvfs_ctx.core_index0_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX0;

	//core index1 map
	gpu_dvfs_ctx.core_index1_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index1_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX1_MAP;
	gpu_dvfs_ctx.core_index1_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX1;

	//core index2 map
	gpu_dvfs_ctx.core_index2_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index2_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX2_MAP;
	gpu_dvfs_ctx.core_index2_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX2;

	//core index3 map
	gpu_dvfs_ctx.core_index3_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index3_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX3_MAP;
	gpu_dvfs_ctx.core_index3_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX3;

	//core index4 map
	gpu_dvfs_ctx.core_index4_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index4_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX4_MAP;
	gpu_dvfs_ctx.core_index4_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX4;

	//core index5 map
	gpu_dvfs_ctx.core_index5_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index5_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX5_MAP;
	gpu_dvfs_ctx.core_index5_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX5;

	//core index6 map
	gpu_dvfs_ctx.core_index6_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index6_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX6_MAP;
	gpu_dvfs_ctx.core_index6_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX6;

	//core index6 map sel
	gpu_dvfs_ctx.core_index6_map_sel.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index6_map_sel.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX6_MAP;
	gpu_dvfs_ctx.core_index6_map_sel.args[1] = MASK_GPU_DVFS_APB_RF_GPU_INDEX6_MAP_SEL;

	//core index7 map
	gpu_dvfs_ctx.core_index7_map.regmap_ptr = gpu_dvfs_ctx.gpu_dvfs_apb_base_ptr;
	gpu_dvfs_ctx.core_index7_map.args[0] = REG_GPU_DVFS_APB_RF_GPU_INDEX7_MAP;
	gpu_dvfs_ctx.core_index7_map.args[1] = MASK_GPU_DVFS_APB_RF_GPU_CORE_VOL_INDEX7;

	//read gpu_bin, 1:FF, 2:TT, 3:SS
	ret = sprd_gpu_cal_read(dev->of_node, "gpu_bin", &gpu_dvfs_ctx.gpu_binning);
	if (ret)
	{
		pr_warn("gpu binning read fail.\n");
	}

	gpu_dvfs_ctx.clk_gpu_i = of_clk_get(dev->of_node, 0);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.clk_gpu_i);

	gpu_dvfs_ctx.clk_gpu_core_eb = of_clk_get(dev->of_node, 1);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.clk_gpu_core_eb);

	clk_cnt = of_clk_get_parent_count(dev->of_node);
	gpu_dvfs_ctx.gpu_clk_num = clk_cnt - DTS_CLK_OFFSET;

	gpu_dvfs_ctx.gpu_clk_src = vmalloc(sizeof(struct clk*) * gpu_dvfs_ctx.gpu_clk_num);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_clk_src);

	for (i = 0; i < gpu_dvfs_ctx.gpu_clk_num; i++)
	{
		gpu_dvfs_ctx.gpu_clk_src[i] = of_clk_get(dev->of_node, i+DTS_CLK_OFFSET);
		KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.gpu_clk_src[i]);
	}

	gpu_dvfs_ctx.freq_list_len = of_property_count_elems_of_size(dev->of_node,"operating-points",2*sizeof(u32));
	gpu_dvfs_ctx.freq_list = vmalloc(sizeof(struct gpu_freq_info) * gpu_dvfs_ctx.freq_list_len);
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.freq_list);

	for(i = 0; i < gpu_dvfs_ctx.freq_list_len; i++)
	{
		gpu_dvfs_ctx.freq_list[i].clk_src = gpu_dvfs_ctx.gpu_clk_src[i];
		KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.freq_list[i].clk_src);
		of_property_read_u32_index(dev->of_node, "operating-points", 2*i,   &gpu_dvfs_ctx.freq_list[i].freq);
		of_property_read_u32_index(dev->of_node, "operating-points", 2*i+1, &gpu_dvfs_ctx.freq_list[i].volt);
		gpu_dvfs_ctx.freq_list[i].dvfs_index = i;
	}
	//adjust freq list
	hwf = of_find_node_by_path("/hwfeature/auto");
	if (hwf) {
		gpu_dvfs_ctx.auto_efuse = of_get_property(hwf, "efuse", NULL);
	}

	//T760 table: 384M, 512M, 650M
	//T770 table: 384M, 512M, 680M, 780M
	//T790 table: 384M, 512M, 680M, 850M
	//default table: 384M, 512M, 680M, 850M
	if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T760"))
	{
		//modify 680M to 650M
		gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-2].freq = T760_GPLL_655M / FREQ_KHZ;

		//remove 850M
		memset(&gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1], 0, sizeof(struct gpu_freq_info));

		//modify freq list len
		gpu_dvfs_ctx.freq_list_len = gpu_dvfs_ctx.freq_list_len -1;
		//compute the gpll freq
		gpll_T760_1638M = T760_GPLL_655M * 2.5;

	}
	else if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770"))
	{
		//copy 650M to 780M and remove 850M
          	memcpy(&gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1],
			&gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-2],
			sizeof(struct gpu_freq_info));
		gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].freq = T770_GPLL_780M / FREQ_KHZ;
		//modify freq index
		gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len -1].dvfs_index = gpu_dvfs_ctx.freq_list_len -1;
		//compute the gpll freq
		gpll_T770_1950M = T770_GPLL_780M * 2.5;
		gpll_T770_1716M = T770_GPLL_686M * 2.5;
	}
	//modify dcdc_voltage
	//0.65v/3.125mv=208->0xD0
	regmap_update_bits(gpu_dvfs_ctx.dcdc_gpu_voltage0.regmap_ptr, gpu_dvfs_ctx.dcdc_gpu_voltage0.args[0], gpu_dvfs_ctx.dcdc_gpu_voltage0.args[1], 0xD0);
	//0.70v/3.125mv=224->0xE0
	regmap_update_bits(gpu_dvfs_ctx.dcdc_gpu_voltage1.regmap_ptr, gpu_dvfs_ctx.dcdc_gpu_voltage1.args[0], gpu_dvfs_ctx.dcdc_gpu_voltage1.args[1], 0xE0<<16);
	//0.75v/3.125mv=240->0xF0
	regmap_update_bits(gpu_dvfs_ctx.dcdc_gpu_voltage2.regmap_ptr, gpu_dvfs_ctx.dcdc_gpu_voltage2.args[0], gpu_dvfs_ctx.dcdc_gpu_voltage2.args[1], 0xF0);
	//0.80v/3.125mv=256->0x100
	regmap_update_bits(gpu_dvfs_ctx.dcdc_gpu_voltage3.regmap_ptr, gpu_dvfs_ctx.dcdc_gpu_voltage3.args[0], gpu_dvfs_ctx.dcdc_gpu_voltage3.args[1], 0x100<<16);

	of_property_read_u32(dev->of_node, "sprd,dvfs-default", &i);
	gpu_dvfs_ctx.freq_default = &gpu_dvfs_ctx.freq_list[i];
	KBASE_DEBUG_ASSERT(gpu_dvfs_ctx.freq_default);

	gpu_dvfs_ctx.cur_index = GPU_MIN_FREQ_INDEX;
	gpu_dvfs_ctx.freq_cur = &gpu_dvfs_ctx.freq_list[GPU_MIN_FREQ_INDEX];
	gpu_dvfs_ctx.cur_voltage = gpu_dvfs_ctx.freq_cur->volt;
}
#endif

#if 0
/* SPRD:gpu_qos_reg_map */
static void gpu_qos_reg_map(void)
{
       u32 qos_offset = 0x100;
       mali_qos_reg_base_mtx_m0 = ioremap(nic400_gpu_sys_mtx_m0_qos_list[0].base_addr,qos_offset);
       mali_qos_reg_base_mtx_m1 = ioremap(nic400_gpu_sys_mtx_m1_qos_list[0].base_addr,qos_offset);
       mali_qos_reg_base_apb_rf = ioremap(gpu_apb_rf_qos_list[0].base_addr,qos_offset);

}
/* SPRD:gpu_qos_reg_unmap */
static void gpu_qos_reg_unmap(void)
{
       iounmap(mali_qos_reg_base_mtx_m0);
       iounmap(mali_qos_reg_base_mtx_m1);
       iounmap(mali_qos_reg_base_apb_rf);
}

//PCIe访问DDR优先级-各个sys modules需要独立管理配置Qos,目前按照要求在各个模块内统一配置
//SPRD note:register 0x23018000 do not support set/clear features, so not use dts reg configuration
#if 1
static void mali_kbase_gpu_sys_qos_reg_rw(struct qos_reg gpu_qos_list_name[], int len)
{
	int i;
	u32 tmp;
        void __iomem *mali_qos_reg_base;

        if(gpu_qos_list_name[0].base_addr == nic400_gpu_sys_mtx_m0_qos_list[0].base_addr)
        {
          mali_qos_reg_base = mali_qos_reg_base_mtx_m0;
        }
        else if(gpu_qos_list_name[0].base_addr == nic400_gpu_sys_mtx_m1_qos_list[0].base_addr)
        {
          mali_qos_reg_base = mali_qos_reg_base_mtx_m1;
        }
        else
        {
          mali_qos_reg_base = mali_qos_reg_base_apb_rf;
        }


	len = len - 1;

	for (i = 0; i < len; i++) {
		tmp = readl_relaxed(mali_qos_reg_base +
				    (gpu_qos_list_name[i].base_addr -
				    gpu_qos_list_name[0].base_addr));
		tmp &= ~(gpu_qos_list_name[i].mask_value);
		tmp |= gpu_qos_list_name[i].set_value;
		writel_relaxed(tmp, mali_qos_reg_base +
			       (gpu_qos_list_name[i].base_addr -
			       gpu_qos_list_name[0].base_addr));
	}
}

static void mali_kbase_gpu_sys_qos_cfg(void)
{
	mali_kbase_gpu_sys_qos_reg_rw(nic400_gpu_sys_mtx_m0_qos_list,
				 sizeof(nic400_gpu_sys_mtx_m0_qos_list) /
				 sizeof(nic400_gpu_sys_mtx_m0_qos_list[0]));
	mali_kbase_gpu_sys_qos_reg_rw(nic400_gpu_sys_mtx_m1_qos_list,
				 sizeof(nic400_gpu_sys_mtx_m1_qos_list) /
				 sizeof(nic400_gpu_sys_mtx_m1_qos_list[0]));
	mali_kbase_gpu_sys_qos_reg_rw(gpu_apb_rf_qos_list,
				 sizeof(gpu_apb_rf_qos_list) /
				 sizeof(gpu_apb_rf_qos_list[0]));
}
#endif
#endif

#if 0
u32 top_pwr = 0;
u32 top_pwr_mask = MOVE_BIT_LEFT(0x1f,16);
u32 top_pwr_wakeup = MOVE_BIT_LEFT(0,16);
#endif

#if 0
static inline int mali_top_state_check(void)
{
	int counter = 0;//count the time of check
	int res = 0;
	//check if the top_state ready or not
	regmap_read(gpu_dvfs_ctx.gpu_top_state_reg.regmap_ptr, gpu_dvfs_ctx.gpu_top_state_reg.args[0], &top_pwr);
	top_pwr = top_pwr & top_pwr_mask;
	while( top_pwr != top_pwr_wakeup)
	{
		udelay(50);
		regmap_read(gpu_dvfs_ctx.gpu_top_state_reg.regmap_ptr, gpu_dvfs_ctx.gpu_top_state_reg.args[0], &top_pwr);
		top_pwr = top_pwr & top_pwr_mask;
		printk(KERN_ERR "SPRDDEBUG gpu_top_pwr = 0x%x, counter = %d\n ", top_pwr, counter);
		if(counter++ > 200)
		{
			printk(KERN_ERR "gpu top power on is timeout ! ");
			WARN_ON(1);
			res = -1;
			return res;
		}

	}
	return res;
}
#endif

int kbase_platform_set_DVFS_table(struct kbase_device *kbdev)
{
#if 0
	//get gpu temperature
	thermal_zone_get_temp(kbdev->gpu_tz, &gpu_dvfs_ctx.gpu_temperature);
	down(gpu_dvfs_ctx.sem);
	if (gpu_dvfs_ctx.gpu_power_on && gpu_dvfs_ctx.gpu_clock_on)
	{
		if ((gpu_dvfs_ctx.last_gpu_temperature <= 65000 && gpu_dvfs_ctx.gpu_temperature >= 65000) || (gpu_dvfs_ctx.last_gpu_temperature >= 65000 && gpu_dvfs_ctx.gpu_temperature <= 65000))
		{
			if (mali_top_state_check() != 0)
			{
				printk(KERN_ERR "mali GPU_set_DVFS_table failed.\n");
				up(gpu_dvfs_ctx.sem);
				return -1;
			}
			else
			{
				printk(KERN_ERR "mali GPU_set_DVFS_table %s gpu_power_on = %d gpu_clock_on = %d,gpu_temperature = %d, last_gpu_temperature = %d.\n",
						__func__, gpu_dvfs_ctx.gpu_power_on, gpu_dvfs_ctx.gpu_clock_on, gpu_dvfs_ctx.gpu_temperature, gpu_dvfs_ctx.last_gpu_temperature);
			}
			if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770"))
			{
				//T770: GPU max freq is 780M, the corresponding GPLL freq is 1950M
				//modify index6_map sel from 6(gpll_850M) to 5(clk_gpll)
				//T770-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-1(0.70v)
				//T770-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-2(0.75v) 65c
				if (1 == gpu_dvfs_ctx.gpu_binning)
				{
					if (gpu_dvfs_ctx.gpu_temperature < 65000)
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 1 << 6);
						gpu_dvfs_ctx.freq_list[6].volt = VOLT_700mV;
					}
					else
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 2 << 6);
						gpu_dvfs_ctx.freq_list[6].volt = VOLT_750mV;
					}
				}
				//T770-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-2(0.75v)
				//T770-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-2(0.75v), index6-780M-2(0.75v) 65c
				else if (2 == gpu_dvfs_ctx.gpu_binning)
				{
					if (gpu_dvfs_ctx.gpu_temperature < 65000)
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
					}
					else
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
					}
				}
				//T770-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-2(0.75v), index6-780M-3(0.80v)
				//T770-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-3(0.80v), index6-780M-3(0.80v) 65C
				else if (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
				{
					if (gpu_dvfs_ctx.gpu_temperature < 65000)
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
					}
					else
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 3 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_800mV;
					}
				}
			}
			else if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T790") || !strcmp(gpu_dvfs_ctx.auto_efuse, "UTS6560S"))
			{
				//T790-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-2(0.75v)
				//T790-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-2(0.75v) 65c
				if (1 == gpu_dvfs_ctx.gpu_binning)
                                {}
				//T790-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-3(0.80v)
				//T790-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-2(0.75v), index6-850M-3(0.80v) 65c
				else if (2 == gpu_dvfs_ctx.gpu_binning)
				{
					if (gpu_dvfs_ctx.gpu_temperature < 65000)
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
					}
					else
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
					}
				}
				//T790-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-2(0.75v), index6-850M-3(0.80v)
				//T790-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-3(0.80v), index6-680M-3(0.80v) 65c
				else if (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
				{
					if (gpu_dvfs_ctx.gpu_temperature < 65000)
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
					}
					else
					{
						regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 3 << 6);
						gpu_dvfs_ctx.freq_list[5].volt = VOLT_800mV;
					}
				}
			}
			gpu_dvfs_ctx.last_gpu_temperature = gpu_dvfs_ctx.gpu_temperature;
		}
	}
	up(gpu_dvfs_ctx.sem);
#endif
	return 0;
}

#if 0
/*gpu_soft_reset function*/
static void gpu_reset_assert(struct reset_control* rst_ctrl)
{
	int ret = 0;
	ret = reset_control_assert(rst_ctrl);
	if (ret)
		printk(KERN_ERR "GPU_RESET %s error = %d\n", __func__, ret);

}

static void gpu_reset_deassert(struct reset_control* rst_ctrl)
{
	int ret = 0;
	ret = reset_control_deassert(rst_ctrl);
	if (ret)
		printk(KERN_ERR "GPU_RESET %s error = %d\n", __func__, ret);

}
#endif

static inline void mali_power_on(void)
{
#if 0
	//gpll force off: 0:not force off; 1:force off
	regmap_update_bits(gpu_dvfs_ctx.gpll_cfg_force_off_reg.regmap_ptr, gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[0], gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[1], ~gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[1]);
	udelay(100);
	//gpll force on: 0:not force on; 1:force on
	regmap_update_bits(gpu_dvfs_ctx.gpll_cfg_force_on_reg.regmap_ptr, gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[0], gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[1], gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[1]);
	udelay(150);

	//gpu regulator enable
	// if (!regulator_is_enabled(gpu_dvfs_ctx.gpu_reg_ptr))
	// {
	// 	regulator_enable(gpu_dvfs_ctx.gpu_reg_ptr);
	// 	udelay(400);
	// }

	regmap_update_bits(gpu_dvfs_ctx.top_force_reg.regmap_ptr, gpu_dvfs_ctx.top_force_reg.args[0], gpu_dvfs_ctx.top_force_reg.args[1], ~gpu_dvfs_ctx.top_force_reg.args[1]);

	//reset gpu sys
	gpu_reset_assert(gpu_dvfs_ctx.gpu_soft_rst);
	udelay(10);
	gpu_reset_deassert(gpu_dvfs_ctx.gpu_soft_rst);
	udelay(300);
	//check if the top_state ready or not
	mali_top_state_check();
#endif

	udelay(100);
	gpu_dvfs_ctx.gpu_power_on = 1;
}


static inline void mali_power_off(void)
{
	gpu_dvfs_ctx.gpu_power_on = 0;
#if 0
	regmap_update_bits(gpu_dvfs_ctx.top_force_reg.regmap_ptr, gpu_dvfs_ctx.top_force_reg.args[0], gpu_dvfs_ctx.top_force_reg.args[1], gpu_dvfs_ctx.top_force_reg.args[1]);

	//gpu regulator disable
	// if (regulator_is_enabled(gpu_dvfs_ctx.gpu_reg_ptr))
	// {
	// 	regulator_disable(gpu_dvfs_ctx.gpu_reg_ptr);
	// 	udelay(10);
	// }

	//gpll force off: 0:not force off; 1:force off
	regmap_update_bits(gpu_dvfs_ctx.gpll_cfg_force_off_reg.regmap_ptr, gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[0], gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[1], gpu_dvfs_ctx.gpll_cfg_force_off_reg.args[1]);
	//gpll force on: 0:not force on; 1:force on
	regmap_update_bits(gpu_dvfs_ctx.gpll_cfg_force_on_reg.regmap_ptr, gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[0], gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[1], ~gpu_dvfs_ctx.gpll_cfg_force_on_reg.args[1]);
#endif
}

#if 0
static void maliQosConfig(void)
{
	regmap_update_bits(gpu_dvfs_ctx.gpu_qos_sel.regmap_ptr, gpu_dvfs_ctx.gpu_qos_sel.args[0], gpu_dvfs_ctx.gpu_qos_sel.args[1], gpu_dvfs_ctx.gpu_qos_sel.args[1]);
	regmap_update_bits(gpu_dvfs_ctx.gpu_qos.regmap_ptr, gpu_dvfs_ctx.gpu_qos.args[0], gpu_dvfs_ctx.gpu_qos.args[1], ((gpu_qos_cfg.awqos_threshold << 12) | (gpu_qos_cfg.arqos_threshold << 8) | (gpu_qos_cfg.awqos << 4) | gpu_qos_cfg.arqos));
}
#endif

static inline void mali_clock_on(void)
{
#if 0
	int i;
	static int clock_on_count = 0;

	//enable all clocks
	for(i = 0; i < gpu_dvfs_ctx.gpu_clk_num; i++)
	{
		clk_prepare_enable(gpu_dvfs_ctx.gpu_clk_src[i]);
	}
	clk_prepare_enable(gpu_dvfs_ctx.clk_gpu_i);

	//enable gpu clock
	clk_prepare_enable(gpu_dvfs_ctx.clk_gpu_core_eb);
	udelay(400);
	//check if the top_state ready or not
	mali_top_state_check();

	//set core index map and clk rate
	//GPU_INDEX_MAP [9:6]default index0-0, index1-0, index2-0, index3-1, index4-2, index5-3,index6-4;
	if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T760") || !strcmp(gpu_dvfs_ctx.auto_efuse, "UTS6560"))
	{
		//T760: GPU max freq is 650M, the corresponding GPLL freq is 1638M
		clk_set_rate(gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].clk_src, gpll_T760_1638M);
		//T760-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-650M-0(0.65v)
		if (1 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[5].volt = VOLT_650mV;
		}
		//T760-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-650M-1(0.70v)
		else if (2 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
		}
		//T760-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-650M-2(0.75v)
		else if (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_700mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_750mV;
		}
	}
	else if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770"))
	{
		//T770: GPU max freq is 780M, the corresponding GPLL freq is 1950M
		//modify index6_map sel from 6(gpll_850M) to 5(clk_gpll)
		regmap_update_bits(gpu_dvfs_ctx.core_index6_map_sel.regmap_ptr, gpu_dvfs_ctx.core_index6_map_sel.args[0], gpu_dvfs_ctx.core_index6_map_sel.args[1], 0x5);
		//T770-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-1(0.70v)
		//T770-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-2(0.75v) 65c
		if (1 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
			if (gpu_dvfs_ctx.gpu_temperature < 65000)
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 1 << 6);
				gpu_dvfs_ctx.freq_list[6].volt = VOLT_700mV;
			}
			else
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 2 << 6);
				gpu_dvfs_ctx.freq_list[6].volt = VOLT_750mV;
			}
		}
		//T770-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-780M-2(0.75v)
		//T770-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-2(0.75v), index6-780M-2(0.75v) 65c
		else if (2 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			if (gpu_dvfs_ctx.gpu_temperature < 65000)
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
			}
			else
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
			}
			regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 2 << 6);
			gpu_dvfs_ctx.freq_list[6].volt = VOLT_750mV;
		}
		//T770-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-2(0.75v), index6-780M-3(0.80v)
		//T770-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-3(0.80v), index6-780M-3(0.80v) 65C
		else if (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_700mV;
			if (gpu_dvfs_ctx.gpu_temperature < 65000)
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
			}
			else
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 3 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_800mV;
			}
			regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 3 << 6);
			gpu_dvfs_ctx.freq_list[6].volt = VOLT_800mV;
		}
	}
	else if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T790") || !strcmp(gpu_dvfs_ctx.auto_efuse, "UTS6560S"))
	{
		//T790-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-2(0.75v)
		//T790-BIN1 FF index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-2(0.75v) 65c
		if (1 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 2 << 6);
			gpu_dvfs_ctx.freq_list[6].volt = VOLT_750mV;
		}
		//T790-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-1(0.70v), index6-850M-3(0.80v)
		//T790-BIN2 TT index3-384M-0(0.65v), index4-512M-0(0.65v), index5-680M-2(0.75v), index6-850M-3(0.80v) 65c
		else if (2 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_650mV;
			if (gpu_dvfs_ctx.gpu_temperature < 65000)
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 1 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_700mV;
			}
			else
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
			}
			regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 3 << 6);
			gpu_dvfs_ctx.freq_list[6].volt = VOLT_800mV;
		}
		//T790-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-2(0.75v), index6-850M-3(0.80v)
		//T790-BIN3 SS index3-384M-0(0.65v), index4-512M-1(0.70v), index5-680M-3(0.80v), index6-680M-3(0.80v) 65c
		else if (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
		{
			regmap_update_bits(gpu_dvfs_ctx.core_index3_map.regmap_ptr, gpu_dvfs_ctx.core_index3_map.args[0], gpu_dvfs_ctx.core_index3_map.args[1], 0 << 6);
			gpu_dvfs_ctx.freq_list[3].volt = VOLT_650mV;
			regmap_update_bits(gpu_dvfs_ctx.core_index4_map.regmap_ptr, gpu_dvfs_ctx.core_index4_map.args[0], gpu_dvfs_ctx.core_index4_map.args[1], 1 << 6);
			gpu_dvfs_ctx.freq_list[4].volt = VOLT_700mV;
			if (gpu_dvfs_ctx.gpu_temperature < 65000)
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 2 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_750mV;
			}
			else
			{
				regmap_update_bits(gpu_dvfs_ctx.core_index5_map.regmap_ptr, gpu_dvfs_ctx.core_index5_map.args[0], gpu_dvfs_ctx.core_index5_map.args[1], 3 << 6);
				gpu_dvfs_ctx.freq_list[5].volt = VOLT_800mV;
			}
			regmap_update_bits(gpu_dvfs_ctx.core_index6_map.regmap_ptr, gpu_dvfs_ctx.core_index6_map.args[0], gpu_dvfs_ctx.core_index6_map.args[1], 3 << 6);
			gpu_dvfs_ctx.freq_list[6].volt = VOLT_800mV;
		}
	}

	//update freq cfg
	regmap_update_bits(gpu_dvfs_ctx.freq_upd_cfg.regmap_ptr, gpu_dvfs_ctx.freq_upd_cfg.args[0], gpu_dvfs_ctx.freq_upd_cfg.args[1], 1);

	//init freq
	regmap_update_bits(gpu_dvfs_ctx.dvfs_index_cfg.regmap_ptr, gpu_dvfs_ctx.dvfs_index_cfg.args[0], gpu_dvfs_ctx.dvfs_index_cfg.args[1], gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.cur_index].dvfs_index);

#if 0
	//qos
	maliQosConfig();
#endif
#if 1
        /* SPRD:qos_reg_map */
        if(clock_on_count == 0)
        {
        gpu_qos_reg_map();
	clock_on_count++;
        }
        //SPRD:PCIe Qos Config gpu_sys_qos_config
        mali_kbase_gpu_sys_qos_cfg();
#endif
#endif
	udelay(200);
	gpu_dvfs_ctx.gpu_clock_on = 1;
}

static inline void mali_clock_off(void)
{
//	int i;

	gpu_dvfs_ctx.gpu_clock_on = 0;
#if 0
	//disable gpu clock
	clk_disable_unprepare(gpu_dvfs_ctx.clk_gpu_core_eb);
	clk_disable_unprepare(gpu_dvfs_ctx.clk_gpu_i);

	//disable all clocks
	for(i = 0; i < gpu_dvfs_ctx.gpu_clk_num; i++)
	{
		clk_disable_unprepare(gpu_dvfs_ctx.gpu_clk_src[i]);
	}
#endif
}

static int mali_platform_init(struct kbase_device *kbdev)
{
	#define REG_RAW_READ(addr)              (*(volatile uint32_t*)(addr))
        #define REG_RAW_WRITE(addr,value)       (*(volatile uint32_t*)(addr) = (value))
        #define REG_RAW_CLEAR(addr,bit)         (*(volatile uint32_t*)(addr) &= ~(1UL<<(bit)))
        #define REG_RAW_SET(addr,bit)           (*(volatile uint32_t*)(addr) |= (1UL<<(bit)))
        #define REG_RAW_OR(addr,value)          (*(volatile uint32_t*)(addr) |= (value))
        #define REG_RAW_AND(addr,value)         (*(volatile uint32_t*)(addr) &= (value))

        uint32_t *GPU_EB = (uint32_t*)ioremap(0x64900000, 4);
        uint32_t *PD_GPU_TOP_FORCE_SHUTDOWN = (uint32_t*)ioremap(0x649102c4, 4);
        uint32_t *CLK_GPU = (uint32_t*)ioremap(0x23010028, 4);
        //uint32_t *CLK_AP_MM = (uint32_t*)ioremap(0x23010040, 4);
        uint32_t *GPLL_REL_CFG = (uint32_t*)ioremap(0x64910a20, 4);
	uint32_t *RESET_GPU = (uint32_t*)ioremap(0x64910b98, 4);
	uint32_t *GPU2EMC_DIV = (uint32_t*)ioremap(0x649203a8, 4); //n6lite:03a8 //n6p:039c
	uint32_t *GPU2EMC_SEL = (uint32_t*)ioremap(0x649203ac, 4); //n6lite:03ac //n6p:03a0
	uint32_t *POWERON_CORE0 = (uint32_t*)ioremap(0x649102c8, 4);
	uint32_t *POWERON_CORE1 = (uint32_t*)ioremap(0x649102cc, 4);
	//uint32_t *POWERON_CORE2 = (uint32_t*)ioremap(0x649102d0, 4);
	//uint32_t *POWERON_CORE3 = (uint32_t*)ioremap(0x649102d4, 4);
	uint32_t *CGM_GPU_CORE_EB = (uint32_t*)ioremap(0x23000000, 4);
	uint32_t *GPU_DVFS_FREQ_UPDATE_BYPASS0 = (uint32_t*)ioremap(0x23014048, 4);
	uint32_t *CGM_GPU_DIV = (uint32_t*)ioremap(0x23010024, 4);
	//uint32_t *MALI_POWER_SECURITY = (uint32_t*)ioremap(0x23100050, 4);

	uint32_t value_GPU_EB, value_PD_GPU_TOP_FORCE_SHUTDOWN, value_CLK_GPU, value_GPLL_REL_CFG, value_RESET_GPU, value_GPU2EMC_DIV, value_GPU2EMC_SEL, value_POWERON_CORE0, value_POWERON_CORE1, value_CGM_GPU_CORE_EB, value_GPU_DVFS_FREQ_UPDATE_BYPASS0, value_CGM_GPU_DIV;
	//value_CLK_AP_MM, value_POWERON_CORE2, value_POWERON_CORE3, value_MALI_POWER_SECURITY

	printk(KERN_INFO "shinian.zhou_mali----start gpu enable\n");

	//enable GPLL for gpu 850M
	REG_RAW_CLEAR(GPLL_REL_CFG, 1);
	udelay(900);
	REG_RAW_SET(GPLL_REL_CFG, 0);
	udelay(900);
	REG_RAW_SET(GPLL_REL_CFG, 2);
	udelay(900);
	printk(KERN_INFO "mali----GPLL_REL_CFG -soft config-01-");

	REG_RAW_CLEAR(PD_GPU_TOP_FORCE_SHUTDOWN, 25);
        udelay(900);
	REG_RAW_SET(GPU_EB, 11);
	mdelay(100);
	printk(KERN_INFO "mali----PD_GPU_TOP_FORCE_SHUTDOWN,GPU_EB -soft config-02-");


	//reset gpu
	REG_RAW_SET(RESET_GPU, 2);
	udelay(900);
	REG_RAW_CLEAR(RESET_GPU,2);
	udelay(900);
	printk(KERN_INFO "mali----RESET_GPU -soft config-03-");

	REG_RAW_WRITE(GPU2EMC_DIV,0x0);
	udelay(900);
	REG_RAW_WRITE(GPU2EMC_SEL,0x6); //the speed of transport data is 856M
	udelay(900);
	printk(KERN_INFO "mali----GPU2EMC_DIV,GPU2EMC_SEL -soft config-04-");

	//power on core0,1,2,3
	REG_RAW_OR(POWERON_CORE0,0x400000f);
	REG_RAW_OR(POWERON_CORE1,0x400000f);
	//REG_RAW_OR(POWERON_CORE2,0x400000f);
	//REG_RAW_OR(POWERON_CORE3,0x400000f);
	mdelay(200);
	printk(KERN_INFO "mali----POWERON_CORE0,2 -soft config-05-");

	REG_RAW_SET(CGM_GPU_CORE_EB,2);
	REG_RAW_SET(GPU_DVFS_FREQ_UPDATE_BYPASS0,0);
	udelay(900);
	printk(KERN_INFO "mali----CGM_GPU_CORE_EB,GPU_DVFS_FREQ_UPDATE_BYPASS0 -soft config-07-");

	REG_RAW_AND(CGM_GPU_DIV,0xFFFFFFF8);
	udelay(900);
	printk(KERN_INFO "mali----CGM_GPU_DIV -soft config-08-");

	REG_RAW_WRITE(CLK_GPU, 0x6);
	udelay(900);
	printk(KERN_INFO "mali----GPLL_REL_CFG -soft config-09-");
	//udelay(900);
	//REG_RAW_WRITE(CLK_AP_MM, 0x2); // gpu read ddr freq,maybe crash 1/5
	//printk(KERN_INFO "mali----CLK_AP_MM -soft config--");
	//mdelay(50);

	//REG_RAW_WRITE(MALI_POWER_SECURITY,0x2968a819);
	//udelay(500);

	REG_RAW_SET(CGM_GPU_CORE_EB,0);
	udelay(500);
	REG_RAW_CLEAR(CGM_GPU_CORE_EB,0);
	mdelay(100);
	printk(KERN_INFO "mali----GPLL_REL_CFG -soft config-10-");


	value_GPLL_REL_CFG = REG_RAW_READ(GPLL_REL_CFG); //0x5
	value_GPU_EB = REG_RAW_READ(GPU_EB);
	value_PD_GPU_TOP_FORCE_SHUTDOWN = REG_RAW_READ(PD_GPU_TOP_FORCE_SHUTDOWN);
	value_CLK_GPU = REG_RAW_READ(CLK_GPU); //0x6
	//value_CLK_AP_MM = REG_RAW_READ(CLK_AP_MM);
	value_RESET_GPU = REG_RAW_READ(RESET_GPU);
	value_GPU2EMC_DIV = REG_RAW_READ(GPU2EMC_DIV);
	value_GPU2EMC_SEL = REG_RAW_READ(GPU2EMC_SEL);
	value_POWERON_CORE0 = REG_RAW_READ(POWERON_CORE0);
	value_POWERON_CORE1 = REG_RAW_READ(POWERON_CORE1);
	//value_POWERON_CORE2 = REG_RAW_READ(POWERON_CORE2);
	//value_POWERON_CORE3 = REG_RAW_READ(POWERON_CORE3);
	value_CGM_GPU_CORE_EB = REG_RAW_READ(CGM_GPU_CORE_EB);
	value_GPU_DVFS_FREQ_UPDATE_BYPASS0 = REG_RAW_READ(GPU_DVFS_FREQ_UPDATE_BYPASS0);
	value_CGM_GPU_DIV = REG_RAW_READ(CGM_GPU_DIV);
	//value_MALI_POWER_SECURITY = REG_RAW_READ(MALI_POWER_SECURITY);


	printk(KERN_INFO "mali----GPLL_REL_CFG(0x64910a20) = 0x%x\n", value_GPLL_REL_CFG);
	printk(KERN_INFO "mali----GPU_EB(0x64900000) = 0x%x\n", value_GPU_EB);//0x2043c880
	printk(KERN_INFO "mali----PD_GPU_TOP_FORCE_SHUTDOWN(0x649102c4) = 0x%x\n", value_PD_GPU_TOP_FORCE_SHUTDOWN);//0x205804
	printk(KERN_INFO "mali----CLK_GPU(0x23010028) = 0x%x\n", value_CLK_GPU);
	//printk(KERN_INFO "mali----CLK_AP_MM(0x23010040) = 0x%x\n", value_CLK_AP_MM);
	printk(KERN_INFO "mali----RESET_GPU (0x64910b98) = 0x%x\n", value_RESET_GPU);
	printk(KERN_INFO "mali----GPU2EMC_DIV(0x6492039c) = 0x%x\n", value_GPU2EMC_DIV);
	printk(KERN_INFO "mali----GPU2EMC_SEL(0x649203a0) = 0x%x\n", value_GPU2EMC_SEL);
	printk(KERN_INFO "mali----POWERON_CORE0(0x649102c8) = 0x%x\n", value_POWERON_CORE0);
	printk(KERN_INFO "mali----POWERON_CORE1(0x649102cc) = 0x%x\n", value_POWERON_CORE1);
	//printk(KERN_INFO "mali----POWERON_CORE2(0x649102d0) = 0x%x\n", value_POWERON_CORE2);
	//printk(KERN_INFO "mali----POWERON_CORE3(0x649102d4) = 0x%x\n", value_POWERON_CORE3);
	printk(KERN_INFO "mali----CGM_GPU_CORE_EB(0x23000000) = 0x%x\n", value_CGM_GPU_CORE_EB);
	printk(KERN_INFO "mali----GPU_DVFS_FREQ_UPDATE_BYPASS0(0x23014048) = 0x%x\n", value_GPU_DVFS_FREQ_UPDATE_BYPASS0);
	printk(KERN_INFO "mali----CGM_GPU_DIV(0x23010024) = 0x%x\n", value_CGM_GPU_DIV);
	//printk(KERN_INFO "mali----MALI_POWER_SECURITY(0x23100050) = 0x%x\n", value_MALI_POWER_SECURITY);

    printk(KERN_INFO "shinian.zhou_mali----end gpu enable\n");

	//gpu freq
//	mali_freq_init(kbdev->dev);
#if 0
#ifdef CONFIG_MALI_DEVFREQ
	InitFreqStats(kbdev);
	kbase_pm_statistics_FreqInit(kbdev);
#endif
#endif
	mali_power_on();

	//clock on
	mali_clock_on();

	return 0;
}

static void mali_platform_term(struct kbase_device *kbdev)
{
//	down(gpu_dvfs_ctx.sem);

	//clock off
	mali_clock_off();

	//power off
	mali_power_off();

#if 0
#ifdef CONFIG_MALI_DEVFREQ
	kbase_pm_statistics_FreqDeinit(kbdev);
	DeinitFreqStats(kbdev);
#endif
        /* SPRD:gpu_qos_unmap */
        gpu_qos_reg_unmap();
	//free
	vfree(gpu_dvfs_ctx.freq_list);
	vfree(gpu_dvfs_ctx.gpu_clk_src);

	up(gpu_dvfs_ctx.sem);
#endif
}

struct kbase_platform_funcs_conf platform_qogirn6l_funcs = {
	.platform_init_func = mali_platform_init,
	.platform_term_func = mali_platform_term
};

#if 0
static void mali_power_mode_change(struct kbase_device *kbdev, int power_mode)
{
	down(gpu_dvfs_ctx.sem);
	//dev_info(kbdev->dev, "mali_power_mode_change: %d, gpu_power_on=%d gpu_clock_on=%d",power_mode,gpu_dvfs_ctx.gpu_power_on,gpu_dvfs_ctx.gpu_clock_on);
	switch (power_mode)
	{
		case 0://power on
			if (!gpu_dvfs_ctx.gpu_power_on)
			{
				mali_power_on();
				mali_clock_on();
			}

			if (!gpu_dvfs_ctx.gpu_clock_on)
			{
				mali_clock_on();
			}
			break;

		case 1://light sleep
		case 2://deep sleep
			if(gpu_dvfs_ctx.gpu_clock_on)
			{
				mali_clock_off();
			}

			if(gpu_dvfs_ctx.gpu_power_on)
			{
				mali_power_off();
			}
			break;

		default:
			break;
	}
	kbase_pm_set_statistics(kbdev, power_mode);
	up(gpu_dvfs_ctx.sem);
}
#endif

static void pm_callback_power_off(struct kbase_device *kbdev)
{
#if 0
#ifdef KBASE_PM_RUNTIME
	int res;

	res = pm_runtime_put_sync(kbdev->dev);
	if (res < 0)
	{
		printk(KERN_ERR "mali----pm_runtime_put_sync return (%d)\n", res);
	}
#endif

	mali_power_mode_change(kbdev, 1);
#endif
}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
#if 0
	mali_power_mode_change(kbdev, 0);

#ifdef KBASE_PM_RUNTIME
	{
		int res;

		res = pm_runtime_get_sync(kbdev->dev);
		if (res < 0)
		{
			printk(KERN_ERR "mali----pm_runtime_get_sync return (%d)\n", res);
		}
	}
#endif
#endif

	return 1;
}

static void pm_callback_power_suspend(struct kbase_device *kbdev)
{
#if 0
	mali_power_mode_change(kbdev, 2);
#endif
}

static void pm_callback_power_resume(struct kbase_device *kbdev)
{
#if 0
	mali_power_mode_change(kbdev, 0);
#endif
}

#ifdef KBASE_PM_RUNTIME
static int pm_callback_power_runtime_init(struct kbase_device *kbdev)
{
#if 0
	pm_runtime_set_active(kbdev->dev);
	pm_suspend_ignore_children(kbdev->dev, true);
	pm_runtime_set_autosuspend_delay(kbdev->dev, PM_RUNTIME_DELAY_MS);
	pm_runtime_use_autosuspend(kbdev->dev);
	pm_runtime_enable(kbdev->dev);
#endif

	return 0;
}

static void pm_callback_power_runtime_term(struct kbase_device *kbdev)
{
	pm_runtime_disable(kbdev->dev);
}
#endif/*CONFIG_PM_RUNTIME*/

struct kbase_pm_callback_conf pm_qogirn6l_callbacks = {
	.power_off_callback = pm_callback_power_off,
	.power_on_callback = pm_callback_power_on,
	.power_suspend_callback = pm_callback_power_suspend,
	.power_resume_callback = pm_callback_power_resume,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = pm_callback_power_runtime_init,
	.power_runtime_term_callback = pm_callback_power_runtime_term,
	.power_runtime_off_callback = NULL,
	.power_runtime_on_callback = NULL
#endif
};


static struct kbase_platform_config versatile_platform_config = {
};

struct kbase_platform_config *kbase_get_platform_config(void)
{
	return &versatile_platform_config;
}

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}

#if 0
#if defined(CONFIG_MALI_DEVFREQ)
static int freq_search(struct gpu_freq_info freq_list[], int len, int key)
{
	int low=0, high=len-1, mid;

	if (0 > key)
	{
		return -1;
	}

	while(low <= high)
	{
		mid = (low+high)/2;
		if(key == freq_list[mid].freq)
		{
			return mid;
		}

		if(key < freq_list[mid].freq)
		{
			high = mid-1;
		}
		else
		{
			low = mid+1;
		}
	}
	return -1;
}
#endif
#endif

int kbase_platform_get_init_freq(void)
{
#if 0
	return (gpu_dvfs_ctx.freq_list[GPU_MIN_FREQ_INDEX].freq * FREQ_KHZ);
#endif
    //gpu_freq = 850Mhz
    return GPU_850M_FREQ;
}

int kbase_platform_get_min_freq(void)
{
#if 0
	return (gpu_dvfs_ctx.freq_list[GPU_MIN_FREQ_INDEX].freq * FREQ_KHZ);
#endif
    //gpu_freq = 850Mhz
    return GPU_850M_FREQ;
}

int kbase_platform_get_max_freq(void)
{
#if 0
	return (gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len -1].freq * FREQ_KHZ);
#endif
    //gpu_freq = 850Mhz
    return GPU_850M_FREQ;
}

void kbase_platform_limit_max_freq(struct device *dev)
{
#if 0
	int ret = 0;

	//disable 26M/76.8M/153.6M
	dev_pm_opp_disable(dev, GPU_26M_FREQ);
	dev_pm_opp_disable(dev, GPU_76M8_FREQ);
	dev_pm_opp_disable(dev, GPU_153M6_FREQ);
	//T760: GPU max freq is 650M
	//T770: GPU max freq is 780M
	//T790: GPU max freq is 850M
	//default table:384M, 512M, 680M, 850M
	if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T760"))
	{
		//T760:384M, 512M, 650M
		//remove 680M and 850M
		dev_pm_opp_disable(dev, GPU_680M_FREQ);
		dev_pm_opp_disable(dev, GPU_850M_FREQ);
		//add GPU max freq 650M
		ret = dev_pm_opp_add(dev, gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].freq * FREQ_KHZ, gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].volt);
		if (ret != 0)
		{
			printk(KERN_ERR "%s: dev_pm_opp_add return %d\n", __func__, ret);
		}
	}
	else if (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770"))
	{
		//default table:384M, 512M, 680M, 780M
		//remove 850M
		dev_pm_opp_disable(dev, GPU_850M_FREQ);
		//add GPU max freq 780M
		ret = dev_pm_opp_add(dev, gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].freq * FREQ_KHZ, gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].volt);
		if (ret != 0)
		{
			printk(KERN_ERR "%s: dev_pm_opp_add return %d\n", __func__, ret);
		}
	}
#endif
}

int kbase_platform_set_freq_volt(int freq, int volt)
{
#if 0
	int index = -1;
	freq = freq/FREQ_KHZ;
	index = freq_search(gpu_dvfs_ctx.freq_list, gpu_dvfs_ctx.freq_list_len, freq);
	printk(KERN_ERR "mali GPU_DVFS %s index = %d cur_freq = %d MHz cur_volt = %d mV --> freq = %d MHz volt = %d mV gpu_power_on = %d gpu_clock_on = %d gpu_temperature = %u.%lu, auto_efuse = %s, gpu_binning = %d.\n",
		__func__, index, gpu_dvfs_ctx.freq_cur->freq / FREQ_KHZ, gpu_dvfs_ctx.cur_voltage / VOLT_KMV, freq / FREQ_KHZ, volt / VOLT_KMV,
		gpu_dvfs_ctx.gpu_power_on, gpu_dvfs_ctx.gpu_clock_on, (unsigned)gpu_dvfs_ctx.gpu_temperature / 1000,
		(unsigned long)gpu_dvfs_ctx.gpu_temperature % 1000, gpu_dvfs_ctx.auto_efuse, gpu_dvfs_ctx.gpu_binning);

	if (0 <= index)
	{
		down(gpu_dvfs_ctx.sem);

		//set frequency
		if (gpu_dvfs_ctx.gpu_power_on && gpu_dvfs_ctx.gpu_clock_on)
		{
			//T770 the clk src of 680MHz and 780MHz are both clk_gpll
			//set gpll VCO 1950M(780M*2.5)
			if (gpu_dvfs_ctx.freq_list_len-1 == index && (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770")))
			{
				clk_set_rate(gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].clk_src, gpll_T770_1950M);
				udelay(50);
			}
			//set gpll VCO 1716M(686.4M*2.5)
			else if (gpu_dvfs_ctx.freq_list_len-2 == index && (!strcmp(gpu_dvfs_ctx.auto_efuse, "T770")))
			{
				clk_set_rate(gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-2].clk_src, gpll_T770_1716M);
				udelay(50);
			}
			//set dvfs index
			regmap_update_bits(gpu_dvfs_ctx.dvfs_index_cfg.regmap_ptr, gpu_dvfs_ctx.dvfs_index_cfg.args[0], gpu_dvfs_ctx.dvfs_index_cfg.args[1], gpu_dvfs_ctx.freq_list[index].dvfs_index);
		}
		gpu_dvfs_ctx.cur_index = index;
		gpu_dvfs_ctx.freq_cur = &gpu_dvfs_ctx.freq_list[index];
		trace_clock_set_rate(VENDOR_FTRACE_MODULE_NAME, gpu_dvfs_ctx.freq_cur->freq, raw_smp_processor_id());
		up(gpu_dvfs_ctx.sem);
	}
#endif
	return 0;
}

#ifdef CONFIG_MALI_BOOST
void kbase_platform_modify_target_freq(struct device *dev, unsigned long *target_freq)
{
#if 0
	int min_index = -1, max_index = -1, modify_flag = 0,user_max_freq, user_min_freq;
	struct gpu_freq_info *freq_max, *freq_min;

	switch(gpu_boost_level)
	{
	case 10:
		freq_max = freq_min = &gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1];
		break;

	case 0:
	default:
		freq_max = &gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1];
		freq_min = &gpu_dvfs_ctx.freq_list[0];
		break;
	}
	user_max_freq = dev_pm_qos_read_value(dev, DEV_PM_QOS_MAX_FREQUENCY);
	user_min_freq = dev_pm_qos_read_value(dev, DEV_PM_QOS_MIN_FREQUENCY);

	//limit min freq
	min_index = freq_search(gpu_dvfs_ctx.freq_list, gpu_dvfs_ctx.freq_list_len, user_min_freq/FREQ_KHZ);
	if ((0 <= min_index) &&
		(freq_min->freq < gpu_dvfs_ctx.freq_list[min_index].freq))
	{
		freq_min = &gpu_dvfs_ctx.freq_list[min_index];
		if (freq_min->freq > freq_max->freq)
		{
			freq_max = freq_min;
		}
	}

	//limit max freq
	max_index = freq_search(gpu_dvfs_ctx.freq_list, gpu_dvfs_ctx.freq_list_len, user_max_freq /FREQ_KHZ);
	if ((0 <= max_index) &&
		(freq_max->freq > gpu_dvfs_ctx.freq_list[max_index].freq))
	{
		freq_max = &gpu_dvfs_ctx.freq_list[max_index];
		if (freq_max->freq < freq_min->freq)
		{
			freq_min = freq_max;
		}
	}
	gpu_boost_level = 0;

	//set target frequency
	if (*target_freq < (unsigned long)freq_min->freq*FREQ_KHZ)
	{
		*target_freq = (unsigned long)freq_min->freq*FREQ_KHZ;
		modify_flag = 1;
	}
	if (*target_freq > (unsigned long)freq_max->freq*FREQ_KHZ)
	{
		*target_freq = (unsigned long)freq_max->freq*FREQ_KHZ;
		modify_flag = 1;
	}
	//T790 SS gpu_temperature >= 65 the max freq of gpu is 680MHz
	if ((*target_freq == (unsigned long)gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-1].freq*FREQ_KHZ ) && (!strcmp(gpu_dvfs_ctx.auto_efuse, "T790") || !strcmp(gpu_dvfs_ctx.auto_efuse, "UTS6560S"))
		&& (3 == gpu_dvfs_ctx.gpu_binning || 0 == gpu_dvfs_ctx.gpu_binning)
		&& gpu_dvfs_ctx.gpu_temperature >= 65000)
	{
		*target_freq = (unsigned long)gpu_dvfs_ctx.freq_list[gpu_dvfs_ctx.freq_list_len-2].freq*FREQ_KHZ;
		modify_flag = 1;
	}
	if(1 == modify_flag)
	{
		printk(KERN_ERR "GPU_DVFS %s gpu_boost_level:%d min_freq=%dMHz max_freq=%dMHz target_freq=%dMHz \n",
			__func__, gpu_boost_level, freq_min->freq / FREQ_KHZ, freq_max->freq / FREQ_KHZ, *target_freq / (FREQ_KHZ * FREQ_KHZ));
	}
#endif
}
#endif

#ifdef CONFIG_MALI_BOOST
void kbase_platform_set_boost(struct kbase_device *kbdev, int boost_level)
{
	if (gpu_boost_level < boost_level)
	{
		gpu_boost_level = boost_level;
		printk(KERN_ERR "GPU_DVFS %s gpu_boost_level =%d \n", __func__, gpu_boost_level);
	}
}
#endif

