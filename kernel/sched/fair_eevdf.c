/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/rbtree_augmented.h>
/*
 * EEVDF accounting — included from fair.c.
 *
 * The rbtree is ordered by virtual deadline. Lag is the only value that
 * survives a migration or a fork; vruntime and deadline are recomputed on
 * the destination cfs_rq. vlag is never used to store a deadline.
 *
 * V is the weighted average of entity vruntimes, excluding the entity whose
 * lag is being measured:
 *
 *   avg_vruntime := \Sum (v_i - min_vruntime) * w_i
 *   avg_load     := \Sum w_i
 */

static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se);

static inline s64 entity_key(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return (s64)(se->vruntime - cfs_rq->min_vruntime);
}

static void avg_vruntime_add(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	unsigned long weight = scale_load_down(se->load.weight);
	s64 key = entity_key(cfs_rq, se);

	cfs_rq->avg_vruntime += key * weight;
	cfs_rq->avg_load += weight;
}

static void avg_vruntime_sub(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	unsigned long weight = scale_load_down(se->load.weight);
	s64 key = entity_key(cfs_rq, se);

	cfs_rq->avg_vruntime -= key * weight;
	cfs_rq->avg_load -= weight;
}

static inline void avg_vruntime_update(struct cfs_rq *cfs_rq, s64 delta)
{
	cfs_rq->avg_vruntime -= cfs_rq->avg_load * delta;
}

/*
 * Weighted average vruntime.
 * @skip:        entity to leave out (NULL keeps everyone).
 * @skip_in_tree: skip is currently inside the stored sum. curr is not in
 *                that sum; it is added here unless it is @skip.
 */
static u64 avg_vruntime_skip(struct cfs_rq *cfs_rq, struct sched_entity *skip,
			     bool skip_in_tree)
{
	struct sched_entity *curr = cfs_rq->curr;
	s64 avg = cfs_rq->avg_vruntime;
	long load = cfs_rq->avg_load;

	if (skip && skip != curr && skip_in_tree) {
		unsigned long weight = scale_load_down(skip->load.weight);

		avg -= entity_key(cfs_rq, skip) * weight;
		load -= weight;
	}

	if (curr && curr->on_rq && curr != skip) {
		unsigned long weight = scale_load_down(curr->load.weight);

		avg += entity_key(cfs_rq, curr) * weight;
		load += weight;
	}

	if (load > 0) {
		if (avg < 0)
			avg -= (load - 1);
		avg = div_s64(avg, load);
	} else {
		avg = 0;
	}

	return cfs_rq->min_vruntime + avg;
}

static u64 avg_vruntime(struct cfs_rq *cfs_rq)
{
	return avg_vruntime_skip(cfs_rq, NULL, false);
}

static int vruntime_eligible(struct cfs_rq *cfs_rq, u64 vruntime)
{
	struct sched_entity *curr = cfs_rq->curr;
	s64 avg = cfs_rq->avg_vruntime;
	long load = cfs_rq->avg_load;

	if (curr && curr->on_rq) {
		unsigned long weight = scale_load_down(curr->load.weight);

		avg += entity_key(cfs_rq, curr) * weight;
		load += weight;
	}

	return avg >= (s64)(vruntime - cfs_rq->min_vruntime) * load;
}

static int entity_eligible(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return vruntime_eligible(cfs_rq, se->vruntime);
}

static s64 entity_lag(u64 avruntime, struct sched_entity *se)
{
	s64 vlag, limit;
	u64 slice = se->slice;

	if (!slice)
		slice = sysctl_sched_min_granularity;

	vlag = (s64)avruntime - (s64)se->vruntime;
	limit = calc_delta_fair(max_t(u64, 2 * slice, TICK_NSEC), se);

	return clamp(vlag, -limit, limit);
}

static void update_entity_lag(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	bool in_tree = se != cfs_rq->curr;

	WARN_ON_ONCE(!se->on_rq);
	se->vlag = entity_lag(avg_vruntime_skip(cfs_rq, se, in_tree), se);
}

static void eevdf_update_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = sched_slice(cfs_rq, se);

	if (!slice)
		slice = sysctl_sched_min_granularity;
	se->slice = slice;
}

static void eevdf_place_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			       int initial)
{
	u64 vruntime = avg_vruntime(cfs_rq);
	s64 lag = 0;

	eevdf_update_slice(cfs_rq, se);

	/*
	 * Scale the preserved lag into the average the entity is about to
	 * join. An empty average (idle rq) keeps the raw lag: dividing by
	 * the entity's own weight would turn a one-slice bonus into a
	 * multi-second one.
	 */
	if (sched_feat(PLACE_LAG) && cfs_rq->nr_running) {
		struct sched_entity *curr = cfs_rq->curr;
		unsigned long load;

		lag = se->vlag;
		load = cfs_rq->avg_load;
		if (curr && curr->on_rq)
			load += scale_load_down(curr->load.weight);

		if (load) {
			lag *= load + scale_load_down(se->load.weight);
			lag = div_s64(lag, load);
		}
	}

	se->vruntime = vruntime - lag;

	{
		u64 vslice = calc_delta_fair(se->slice, se);

		if (sched_feat(PLACE_DEADLINE_INITIAL) && initial)
			vslice /= 2;
		if (!vslice)
			vslice = 1;
		se->deadline = se->vruntime + vslice;
	}
}

static void clear_buddies(struct cfs_rq *cfs_rq, struct sched_entity *se);

static void update_deadline(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se->deadline && (s64)(se->vruntime - se->deadline) < 0)
		return;

	eevdf_update_slice(cfs_rq, se);
	se->deadline = se->vruntime + calc_delta_fair(se->slice, se);
	if (se->deadline == se->vruntime)
		se->deadline++;

	if (cfs_rq->nr_running > 1) {
		resched_curr(rq_of(cfs_rq));
		clear_buddies(cfs_rq, se);
	}
}

#define __node_2_se(node) rb_entry((node), struct sched_entity, run_node)

static struct sched_entity *__pick_root_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *root = cfs_rq->tasks_timeline.rb_root.rb_node;

	if (!root)
		return NULL;
	return __node_2_se(root);
}

static inline u64 compute_min_vruntime(struct sched_entity *se)
{
	struct rb_node *node = &se->run_node;
	u64 min_v = se->vruntime;

	if (node->rb_left)
		min_v = min_vruntime(min_v,
			__node_2_se(node->rb_left)->min_vruntime);
	if (node->rb_right)
		min_v = min_vruntime(min_v,
			__node_2_se(node->rb_right)->min_vruntime);
	return min_v;
}

RB_DECLARE_CALLBACKS(static, min_vruntime_cb, struct sched_entity,
		     run_node, u64, min_vruntime, compute_min_vruntime);

static void eevdf_enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct rb_node **link = &cfs_rq->tasks_timeline.rb_root.rb_node;
	struct rb_node *parent = NULL;
	struct sched_entity *entry;
	bool leftmost = true;

	if (!se->slice)
		se->slice = sysctl_sched_min_granularity;
	if (!se->deadline)
		se->deadline = se->vruntime + calc_delta_fair(se->slice, se);

	avg_vruntime_add(cfs_rq, se);
	se->min_vruntime = se->vruntime;

	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct sched_entity, run_node);
		if ((s64)(se->deadline - entry->deadline) < 0) {
			link = &parent->rb_left;
		} else {
			link = &parent->rb_right;
			leftmost = false;
		}
	}

	rb_link_node(&se->run_node, parent, link);
	rb_insert_augmented_cached(&se->run_node, &cfs_rq->tasks_timeline,
				   leftmost, &min_vruntime_cb);
}

static void eevdf_dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	rb_erase_augmented_cached(&se->run_node, &cfs_rq->tasks_timeline,
				  &min_vruntime_cb);
	avg_vruntime_sub(cfs_rq, se);
}

static bool eevdf_slice_unexpired(struct sched_entity *se)
{
	return se->deadline && (s64)(se->vruntime - se->deadline) < 0;
}

/*
 * @parity: honor RUN_TO_PARITY. Wakeup preemption passes false so a newly
 * woken eligible task is not stuck behind the rest of the current slice.
 * The tick path passes true.
 */
static struct sched_entity *__pick_eevdf(struct cfs_rq *cfs_rq, bool parity)
{
	struct rb_node *node = cfs_rq->tasks_timeline.rb_root.rb_node;
	struct sched_entity *se = __pick_first_entity(cfs_rq);
	struct sched_entity *curr = cfs_rq->curr;
	struct sched_entity *best = NULL;

	if (cfs_rq->nr_running == 1)
		return curr && curr->on_rq ? curr : se;

	if (curr && (!curr->on_rq || !entity_eligible(cfs_rq, curr)))
		curr = NULL;

	/*
	 * RUN_TO_PARITY keeps curr only when nobody queued has a sooner
	 * deadline. An earlier deadline (a woken interactive task) still
	 * preempts, including from pick_next after check_preempt.
	 */
	if (parity && sched_feat(RUN_TO_PARITY) && curr &&
	    eevdf_slice_unexpired(curr) &&
	    (!se || (s64)(se->deadline - curr->deadline) >= 0))
		return curr;

	if (se && entity_eligible(cfs_rq, se)) {
		best = se;
		goto found;
	}

	while (node) {
		struct rb_node *left = node->rb_left;

		if (left && vruntime_eligible(cfs_rq,
				__node_2_se(left)->min_vruntime)) {
			node = left;
			continue;
		}

		se = __node_2_se(node);
		if (entity_eligible(cfs_rq, se)) {
			best = se;
			break;
		}
		node = node->rb_right;
	}
found:
	/*
	 * curr was already filtered to eligible. Prefer it when it has no
	 * queued peer or its deadline is strictly earlier.
	 */
	if (curr && curr->deadline &&
	    (!best || (s64)(curr->deadline - best->deadline) < 0))
		best = curr;
	if (!best)
		best = __pick_first_entity(cfs_rq);

	return best;
}

static struct sched_entity *pick_eevdf(struct cfs_rq *cfs_rq)
{
	return __pick_eevdf(cfs_rq, true);
}

static void reweight_eevdf(struct sched_entity *se, u64 avruntime,
			   unsigned long weight)
{
	unsigned long old_weight = se->load.weight;
	s64 vlag, vslice = 0;

	if (!weight || !old_weight)
		return;

	/*
	 * Remaining virtual slice, taken before vruntime moves.
	 * Scaling (deadline - avg) instead mixes lag into the slice and
	 * the two divisions round differently.
	 */
	if (se->deadline)
		vslice = (s64)(se->deadline - se->vruntime);

	if (avruntime != se->vruntime) {
		vlag = entity_lag(avruntime, se);
		vlag = div_s64(vlag * (s64)old_weight, (s64)weight);
		se->vruntime = avruntime - vlag;
	}

	if (!se->deadline)
		return;

	vslice = div_s64(vslice * (s64)old_weight, (s64)weight);
	if (vslice < 1)
		vslice = 1;
	se->deadline = se->vruntime + vslice;
}

void sched_eevdf_apply_slice(struct sched_entity *se, u64 slice)
{
	u64 vslice;

	if (!slice)
		slice = sysctl_sched_min_granularity;
	se->slice = slice;
	vslice = calc_delta_fair(slice, se);
	if (!se->deadline ||
	    (s64)(se->deadline - se->vruntime) > (s64)vslice)
		se->deadline = se->vruntime + vslice;
}
