/* arch/arm/mach-msm/pmic_global.c
 *
 * Author: Markinus
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#include "pmic_global.h"

#include <mach/msm_rpcrouter.h>
#include <mach/amss_para.h>

/* rpc related */
#define PM_RPC_TIMEOUT (5*HZ)

#define PM_RPC_PROG	0x30000060
#define PM_RPC_VER	0x00010001

#define PM_RPC_PROC_SET_VREG	(0x3)


/* error bit flags defined by modem side */
#define PM_ERR_FLAG__PAR1_OUT_OF_RANGE		(0x0001)
#define PM_ERR_FLAG__PAR2_OUT_OF_RANGE		(0x0002)
#define PM_ERR_FLAG__PAR3_OUT_OF_RANGE		(0x0004)
#define PM_ERR_FLAG__PAR4_OUT_OF_RANGE		(0x0008)
#define PM_ERR_FLAG__PAR5_OUT_OF_RANGE		(0x0010)

#define PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE   	(0x001F)

#define PM_ERR_FLAG__SBI_OPT_ERR		(0x0080)
#define PM_ERR_FLAG__FEATURE_NOT_SUPPORTED	(0x0100)

#define	PM_BUFF_SIZE		256

static DEFINE_MUTEX(global_pm_mutex);
static struct msm_rpc_endpoint *pm_ept;


static int modem_to_linux_err(uint err)
{
	if (err == 0)
		return 0;
	pr_err("RPC ERROR: %x\n", err);
	if (err & PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE)
		return -EINVAL;

	if (err & PM_ERR_FLAG__SBI_OPT_ERR)
		return -EIO;

	if (err & PM_ERR_FLAG__FEATURE_NOT_SUPPORTED)
		return -ENOSYS;

	return -EPERM;
}


/*
 * 1) network byte order
 * 2) RPC request header(40 bytes) and RPC reply header (24 bytes)
 * 3) each transaction consists of a request and reply
 * 3) PROC (comamnd) layer has its own sub-protocol defined
 * 4) sub-protocol can be grouped to follwoing 1 case:
 * 	set three argument
  */

/* Returns number of reply bytes (minus reply header size) or
 * negative value on error.
 */
static int pm_rpc(int proc, void *msg, int msglen, void *rep, int replen)
{
	int r;
	mutex_lock(&global_pm_mutex);

	if (!pm_ept) {
		pm_ept = msm_rpc_connect(PM_RPC_PROG, PM_RPC_VER, 0);
		if (!pm_ept) {
			pr_err("pmic: cannot connect to rpc server\n");
			r = -ENODEV;
			goto done;
		}
	}
	r = msm_rpc_call_reply(pm_ept, proc, msg, msglen, 
			       rep, replen, PM_RPC_TIMEOUT);
	if (r >= 0) {
		if (r < sizeof(struct rpc_reply_hdr)) {
			r = -EIO;
			goto done;
		}
		r -= sizeof(struct rpc_reply_hdr);
	}
done:
	mutex_unlock(&global_pm_mutex);
	return r;
}

struct pm_reply {
	struct rpc_reply_hdr hdr;
	uint32_t status;
	uint32_t data;
};
	
/**
 * pm_rpc_set_only() - set arguments and no get
 * @data0:	first argumrnt
 * @data1:	second argument
 * @data2:	third argument
 * @num:	number of argument
 * @proc:	command/request id
 *
 */
static int pm_rpc_set_only(uint data0, uint data1, uint data2, uint num, uint proc)
{
	struct {
		struct rpc_request_hdr hdr;
		uint32_t data[3];
	} msg;
	
	struct pm_reply rep;
	int r;

	if (num > 3)
		return -EINVAL;

	msg.data[0] = cpu_to_be32(data0);
	msg.data[1] = cpu_to_be32(data1);
	msg.data[2] = cpu_to_be32(data2);

	r = pm_rpc(proc, &msg,
		     sizeof(struct rpc_request_hdr) + num * sizeof(uint32_t),
		     &rep, sizeof(rep));
	if (r < 0)
		return 0;
	if (r < sizeof(uint32_t))
		return -EIO;

	return modem_to_linux_err(be32_to_cpu(rep.status));
}


int pmic_glb_set_vreg(int enable, enum vreg_id id)
{
	switch(__amss_version) {
		case 5225:
		case 6125:
		case 6150:
			id = 1U << id;
			if(enable) {
				return dex_comm(DEX_PMIC_REG_ON, &id, 0);
			}
			else {
				return dex_comm(DEX_PMIC_REG_OFF, &id, 0);
			}
		break;
		default:
			return msm_proc_comm(PCOM_VREG_SWITCH, &id, &enable);
		break;
	}
}

int pmic_glb_vreg_set_level(enum vreg_id id, unsigned millivolt)
{
	switch(__amss_version) {
		case 5225:
		case 6125:
		case 6150:
			id = 1U << id;
			return dex_comm(DEX_PMIC_REG_VOLTAGE, &id, &millivolt);
		break;
		default:
			return msm_proc_comm(PCOM_VREG_SET_LEVEL, &id, &millivolt);
		break;
	}
}

int pmic_glb_power_down()
{
	switch(__amss_version) {
		case 1550:
		case 5225:
		case 6125:
		case 6150:
			return dex_comm(DEX_POWER_OFF, 0, 0);
		break;
		default:
			return msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
		break;
	}
}

int pmic_glb_reset_chip(unsigned restart_reason)
{
	switch(__amss_version) {
		case 1550:
		case 5225:
		case 6125:
		case 6150:
			printk(KERN_ERR "No msm_hw_reset_hook() available! System halted.\n");
			return 0;
		break;
		default:
			return msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
		break;
	}
}


