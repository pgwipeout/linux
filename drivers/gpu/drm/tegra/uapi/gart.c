/* SPDX-License-Identifier: GPL-2.0 */

#include "drm.h"
#include "gart.h"
#include "job.h"

static inline bool
tegra_bo_mm_evict_bo(struct tegra_drm *tegra, struct tegra_bo *bo,
		     bool release, bool unmap, bool sanitize)
{
	size_t bytes;

	if (list_empty(&bo->mm_eviction_entry))
		return false;

	if (unmap) {
		bytes = iommu_unmap(tegra->domain, bo->gartaddr, bo->iosize);
		if (bytes != bo->iosize)
			DRM_ERROR("failed to unmap bo\n");
	}

	if (release)
		drm_mm_remove_node(&bo->mm);

	if (sanitize)
		bo->gartaddr = 0x666DEAD0;

	list_del_init(&bo->mm_eviction_entry);

	return true;
}

static inline void
tegra_bo_mm_release_victims(struct tegra_drm *tegra,
			    struct list_head *victims_list,
			    bool cleanup,
			    dma_addr_t start,
			    dma_addr_t end)
{
	dma_addr_t victim_start;
	dma_addr_t victim_end;
	struct tegra_bo *tmp;
	struct tegra_bo *bo;
	size_t bytes;
	size_t size;


	/*
	 * Remove BO from MM and unmap only part of BO that is outside of
	 * the given [star, end) range. The overlapping region will be mapped
	 * by a new BO shortly, this reduces re-mapping overhead.
	 */
	list_for_each_entry_safe(bo, tmp, victims_list, mm_eviction_entry) {
		if (!cleanup) {
			victim_end = bo->gartaddr + bo->iosize;
			victim_start = bo->gartaddr;

			if (victim_start < start) {
				size = start - victim_start;

				bytes = iommu_unmap(tegra->domain,
						    victim_start, size);
				if (bytes != size)
					DRM_ERROR("failed to unmap bo\n");
			}

			if (victim_end > end) {
				size = victim_end - end;

				bytes = iommu_unmap(tegra->domain, end, size);
				if (bytes != size)
					DRM_ERROR("failed to unmap bo\n");
			}
		}

		tegra_bo_mm_evict_bo(tegra, bo, false, cleanup, true);
	}
}

static inline bool
tegra_bo_mm_evict_something(struct tegra_drm *tegra,
			    struct list_head *victims_list,
			    size_t size)
{
	LIST_HEAD(scan_list);
	struct list_head *eviction_list;
	struct drm_mm_scan scan;
	struct tegra_bo *tmp;
	struct tegra_bo *bo;
	unsigned long order;
	bool found = false;

	eviction_list = &tegra->mm_eviction_list;
	order = __ffs(tegra->domain->pgsize_bitmap);

	if (list_empty(eviction_list))
		return false;

	drm_mm_scan_init(&scan, &tegra->mm, size,
			 1UL << order, 0, DRM_MM_INSERT_BEST);

	list_for_each_entry_safe(bo, tmp, eviction_list, mm_eviction_entry) {
		/* move BO from eviction to scan list */
		list_move(&bo->mm_eviction_entry, &scan_list);

		/* check whether hole has been found */
		if (drm_mm_scan_add_block(&scan, &bo->mm)) {
			found = true;
			break;
		}
	}

	list_for_each_entry_safe(bo, tmp, &scan_list, mm_eviction_entry) {
		/*
		 * We can't release BO's mm node here, see comments to
		 * drm_mm_scan_remove_block() in drm_mm.c
		 */
		if (drm_mm_scan_remove_block(&scan, &bo->mm))
			list_move(&bo->mm_eviction_entry, victims_list);
		else
			list_move(&bo->mm_eviction_entry, eviction_list);
	}

	/*
	 * Victims would be unmapped later, only mark them as released
	 * for now.
	 */
	list_for_each_entry(bo, victims_list, mm_eviction_entry)
		drm_mm_remove_node(&bo->mm);

	return found;
}

/*
 * GART's aperture has a limited size of 32MB and we want to avoid frequent
 * remappings. To reduce the number of remappings, the mappings are not
 * getting released (say stay in cache) until there is no space in the GART
 * or BO is destroyed. Once there is not enough space for the mapping, the
 * DRM's MM scans mappings for a suitable hole and tells what cached mappings
 * should be released in order to free up enough space for the mapping to
 * succeed.
 */
static int tegra_bo_gart_map_locked(struct tegra_drm *tegra,
				    struct tegra_bo *bo,
				    bool enospc_fatal)
{
	unsigned long order = __ffs(tegra->domain->pgsize_bitmap);
	LIST_HEAD(victims_list);
	size_t gart_size;
	int ret;

	/* check whether BO is already mapped */
	if (bo->iomap_cnt++)
		return 0;

	/* if BO is on the eviction list, just remove it from the list */
	if (tegra_bo_mm_evict_bo(tegra, bo, false, false, false))
		return 0;

	/* BO shall not be mapped from other places */
	WARN_ON_ONCE(drm_mm_node_allocated(&bo->mm));

	ret = drm_mm_insert_node_generic(&tegra->mm, &bo->mm, bo->gem.size,
					 1UL << order, 0, DRM_MM_INSERT_BEST);
	if (!ret)
		goto mm_ok;

	/*
	 * If there is not enough room in GART, release cached mappings
	 * and try again. Otherwise error out.
	 */
	if (ret != -ENOSPC)
		goto mm_err;

	gart_size = tegra->domain->geometry.aperture_end + 1 -
		    tegra->domain->geometry.aperture_start;

	/* check whether BO could be squeezed into GART at all */
	if (bo->gem.size > gart_size) {
		ret = enospc_fatal ? -ENOMEM : -ENOSPC;
		goto mm_err;
	}

	/*
	 * Scan for a suitable hole conjointly with a cached mappings
	 * and release mappings from cache if needed.
	 */
	if (!tegra_bo_mm_evict_something(tegra, &victims_list, bo->gem.size))
		goto mm_err;

	/*
	 * We have freed some of the cached mappings and now reservation
	 * should succeed.
	 */
	ret = drm_mm_insert_node_generic(&tegra->mm, &bo->mm, bo->gem.size,
					 1UL << order, 0, DRM_MM_INSERT_BEST);
	if (ret)
		goto mm_err;

mm_ok:
	bo->gartaddr = bo->mm.start;

	bo->iosize = iommu_map_sg(tegra->domain, bo->gartaddr, bo->sgt->sgl,
				  bo->sgt->nents, IOMMU_READ | IOMMU_WRITE);
	if (!bo->iosize) {
		DRM_ERROR("gart mapping failed\n");
		drm_mm_remove_node(&bo->mm);
		ret = -ENOMEM;
	}

mm_err:
	if (ret) {
		if (ret != -ENOSPC || drm_debug_enabled(DRM_UT_DRIVER))
			DRM_ERROR("failed size %zu: %d\n", bo->gem.size, ret);

		bo->gartaddr = 0x666DEAD0;
		bo->iomap_cnt = 0;

		/* nuke all affected victims */
		tegra_bo_mm_release_victims(tegra, &victims_list, true, 0, 0);
	} else {
		/*
		 * Unmap all affected victims, excluding the newly mapped
		 * BO range.
		 */
		tegra_bo_mm_release_victims(tegra, &victims_list, false,
					    bo->gartaddr,
					    bo->gartaddr + bo->iosize);
	}

	return ret;
}

void tegra_bo_gart_unmap_locked(struct tegra_drm *tegra, struct tegra_bo *bo)
{
	bool on_eviction_list = !list_empty(&bo->mm_eviction_entry);

	if (!on_eviction_list && !bo->iomap_cnt)
		return;

	WARN_ONCE(bo->iomap_cnt, "imbalanced bo unmapping\n");

	/* put mapping into the eviction cache */
	if (!on_eviction_list && !bo->iomap_cnt)
		list_add(&bo->mm_eviction_entry, &tegra->mm_eviction_list);

	tegra_bo_mm_evict_bo(tegra, bo, true, true, true);
}

static inline void
tegra_bo_gart_unmap_cached(struct tegra_drm *tegra, struct tegra_bo *bo)
{
	WARN_ONCE(!bo->iomap_cnt, "imbalanced bo unmapping\n");

	/* put mapping into the eviction cache */
	if (--bo->iomap_cnt == 0)
		list_add(&bo->mm_eviction_entry, &tegra->mm_eviction_list);
}

void tegra_drm_job_unmap_gart_locked(struct tegra_drm *tegra,
				     struct tegra_bo **bos,
				     unsigned int num_bos,
				     unsigned long *bos_gart_bitmap)
{
	unsigned int i;

	for_each_set_bit(i, bos_gart_bitmap, num_bos)
		tegra_bo_gart_unmap_cached(tegra, bos[i]);

	bitmap_clear(bos_gart_bitmap, 0, num_bos);
}

static inline int
tegra_drm_job_pre_check_gart_space(struct tegra_drm *tegra,
				   struct tegra_bo **bos,
				   unsigned int num_bos,
				   unsigned long *bos_gart_bitmap)
{
	struct drm_mm_node *mm;
	struct tegra_bo *bo;
	size_t sparse_size = 0;
	size_t gart_size;
	unsigned int i;
	bool mapped;

	for (i = 0; i < num_bos; i++) {
		bo = bos[i];

		/* all job's BO's must be unmapped now */
		mapped = test_bit(i, bos_gart_bitmap);
		if (WARN_ON_ONCE(mapped))
			return -EINVAL;

		/* gathers are a property of host1x */
		if (bo->flags & TEGRA_BO_HOST1X_GATHER)
			continue;

		if (bo->sgt->nents > 1)
			sparse_size += bo->gem.size;
	}

	/* no sparse BO's? good, we're done */
	if (!sparse_size)
		return 0;

	gart_size = tegra->domain->geometry.aperture_end + 1 -
		    tegra->domain->geometry.aperture_start;

	/*
	 * If total size of sparse allocations is larger than the
	 * GART's  aperture, then there is nothing we could do about it.
	 *
	 * Userspace need to take that into account.
	 */
	if (sparse_size > gart_size)
		return -ENOMEM;

	/*
	 * Get idea about the free space by not taking into account memory
	 * fragmentation.
	 */
	drm_mm_for_each_node(mm, &tegra->mm) {
		bo = container_of(mm, struct tegra_bo, mm);

		if (!list_empty(&bo->mm_eviction_entry))
			gart_size -= bo->gem.size;
	}

	/*
	 * No way allocation could succeed if non-fragmented space is
	 * smaller than the needed amount.
	 */
	if (gart_size < sparse_size)
		return -ENOSPC;

	return 0;
}

/*
 * Map job BO's into the GART aperture. Due to limited size of the aperture,
 * mapping of contiguous allocations is optional and we're trying to map
 * everything till no aperture space left. Mapping of scattered allocations
 * is mandatory because there is no other way to handle these allocations.
 * If there is not enough space in GART, then all succeeded mappings are
 * unmapped and caller should try again after "gart_free_up" completion is
 * signalled. Note that GART doesn't make system secure and only improves
 * system stability by providing some optional protection for memory from a
 * badly-behaving hardware.
 */
int tegra_drm_job_map_gart_locked(struct tegra_drm *tegra,
				  struct tegra_bo **bos,
				  unsigned int num_bos,
				  unsigned long *bos_write_bitmap,
				  unsigned long *bos_gart_bitmap)
{
	struct tegra_bo *bo;
	unsigned int i;
	int err;

	/* quickly check whether job could be handled by GART at all */
	err = tegra_drm_job_pre_check_gart_space(tegra, bos, num_bos,
						 bos_gart_bitmap);
	if (err) {
		if (err == -ENOSPC)
			goto err_retry;

		return err;
	}

	/* map all scattered BO's, this must not fail */
	for (i = 0; i < num_bos; i++) {
		bo = bos[i];

		/* gathers are a property of host1x */
		if (bo->flags & TEGRA_BO_HOST1X_GATHER)
			continue;

		if (bo->sgt->nents > 1) {
			err = tegra_bo_gart_map_locked(tegra, bo, true);
			if (err)
				goto err_unmap;

			set_bit(i, bos_gart_bitmap);
		}
	}

	/* then map the writable BO's */
	for_each_set_bit(i, bos_write_bitmap, num_bos) {
		bo = bos[i];

		/* go next if already mapped */
		if (test_bit(i, bos_gart_bitmap))
			continue;

		/* gathers are a property of host1x */
		if (bo->flags & TEGRA_BO_HOST1X_GATHER)
			continue;

		/* go next if GART has no space */
		err = tegra_bo_gart_map_locked(tegra, bo, false);
		if (err == -ENOSPC)
			continue;

		if (err)
			goto err_unmap;

		set_bit(i, bos_gart_bitmap);
	}

	/*
	 * Mapping of read-only BO's is optional, skip this phase to save
	 * some IOVA space.
	 */
	if (!drm_debug_enabled(DRM_UT_DRIVER))
		return 0;

	/* then map the read-only BO's */
	for (i = 0; i < num_bos; i++) {
		bo = bos[i];

		/* go next if already mapped */
		if (test_bit(i, bos_gart_bitmap))
			continue;

		/* gathers are a property of host1x */
		if (bo->flags & TEGRA_BO_HOST1X_GATHER)
			continue;

		/* go next if GART has no space */
		err = tegra_bo_gart_map_locked(tegra, bo, false);
		if (err == -ENOSPC)
			continue;

		if (err)
			goto err_unmap;

		set_bit(i, bos_gart_bitmap);
	}

	return 0;

err_unmap:
	tegra_drm_job_unmap_gart_locked(tegra, bos, num_bos, bos_gart_bitmap);

	/*
	 * Caller should retry if GART has no space, but allocation could
	 * succeed after freeing some space.
	 */
	if (err == -ENOSPC) {
err_retry:
		reinit_completion(&tegra->gart_free_up);
		return -EAGAIN;
	}

	return err;
}
