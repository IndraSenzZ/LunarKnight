/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BORE on EEVDF used to reweight the task (nice + shorter slice) on every
 * burst update. That stacked a second penalty on top of calc_delta_fair_bore()
 * and rewrote PELT / load-balance weight underneath schedtune and uclamp.
 * The vruntime charge is the only penalty now. This symbol stays so the
 * update/fork/yield hooks in bore.c keep a single call site.
 */

static void bore_eevdf_commit(struct task_struct *p)
{
	(void)p;
}
