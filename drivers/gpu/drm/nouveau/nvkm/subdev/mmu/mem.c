/*
 * Copyright 2017 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#define nvkm_mem(p) container_of((p), struct nvkm_mem, memory)
#include "mem.h"

#include <core/memory.h>
#include <core/tegra.h>
#include <subdev/fb.h>

#include <nvif/if000a.h>
#include <nvif/unpack.h>

#include <linux/iommu.h>

struct nvkm_mem {
	struct nvkm_memory memory;
	enum nvkm_memory_target target;
	struct nvkm_mmu *mmu;
	u64 pages;
	u8 page;
	struct page **mem;
	union {
		struct scatterlist *sgl;
		dma_addr_t *dma;
	};

	/* Tegra: IOMMU-flattened view of the above, for a page > PAGE_SIZE */
	struct nvkm_mm_node *iommu;
	struct nvkm_mm_node node;
};

static enum nvkm_memory_target
nvkm_mem_target(struct nvkm_memory *memory)
{
	return nvkm_mem(memory)->target;
}

static u8
nvkm_mem_page(struct nvkm_memory *memory)
{
	return nvkm_mem(memory)->page;
}

static u64
nvkm_mem_addr(struct nvkm_memory *memory)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	if (mem->iommu)
		return (u64)mem->node.offset << NVKM_RAM_MM_SHIFT;
	if (mem->pages == 1 && mem->mem)
		return mem->dma[0];
	return ~0ULL;
}

static u64
nvkm_mem_size(struct nvkm_memory *memory)
{
	return nvkm_mem(memory)->pages << PAGE_SHIFT;
}

static int
nvkm_mem_map_dma(struct nvkm_memory *memory, u64 offset, struct nvkm_vmm *vmm,
		 struct nvkm_vma *vma, void *argv, u32 argc)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	struct nvkm_vmm_map map = {
		.memory = &mem->memory,
		.offset = offset,
		.dma = mem->dma,
	};
	return nvkm_vmm_map(vmm, vma, argv, argc, &map);
}

static int
nvkm_mem_map_iommu(struct nvkm_memory *memory, u64 offset, struct nvkm_vmm *vmm,
		   struct nvkm_vma *vma, void *argv, u32 argc)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	struct nvkm_vmm_map map = {
		.memory = &mem->memory,
		.offset = offset,
		.mem = &mem->node,
	};
	return nvkm_vmm_map(vmm, vma, argv, argc, &map);
}

static struct nvkm_device_tegra *
nvkm_mem_tegra(struct nvkm_mmu *mmu)
{
	struct nvkm_device *device = mmu->subdev.device;
	struct nvkm_device_tegra *tdev;

	if (!device->func->tegra)
		return NULL;

	tdev = device->func->tegra(device);
	if (!tdev || !tdev->iommu.domain || !tdev->func->iommu_bit ||
	    tdev->iommu.pgshift != PAGE_SHIFT)
		return NULL;

	/* dma[] goes into the IOMMU as a physical address: dma-direct only. */
	if (dev_dma_iommu(device->dev) || device->dev->dma_range_map)
		return NULL;

	return tdev;
}

bool
nvkm_mem_iommu_usable(struct nvkm_mmu *mmu)
{
	return !!nvkm_mem_tegra(mmu);
}

/* Called with iommu.mutex held; the first "mapped" pages are translated. */
static void
nvkm_mem_iommu_put(struct nvkm_device_tegra *tdev, struct nvkm_mem *mem,
		   u64 mapped)
{
	struct nvkm_memory_keep *keep = mem->memory.keep;
	struct nvkm_mm_node **piova = keep ? &keep->iova : &mem->iommu;

	/* Unless a newer memory object of the buffer has taken the range over. */
	if (!keep || keep->user == mem) {
		iommu_unmap(tdev->iommu.domain,
			    (u64)(*piova)->offset << tdev->iommu.pgshift,
			    mapped << PAGE_SHIFT);
		if (keep)
			keep->user = NULL;
	}

	/* The tags are only valid at this address: keep it with them. */
	if (!keep || (!keep->tags && !keep->user))
		nvkm_mm_free(&tdev->iommu.mm, piova);
	mem->iommu = NULL;
}

static void
nvkm_mem_iommu_fini(struct nvkm_mem *mem)
{
	struct nvkm_device_tegra *tdev = nvkm_mem_tegra(mem->mmu);

	mutex_lock(&tdev->iommu.mutex);
	nvkm_mem_iommu_put(tdev, mem, mem->pages);
	mutex_unlock(&tdev->iommu.mutex);
}

void
nvkm_mem_keep_put(struct nvkm_memory_keep **pkeep)
{
	struct nvkm_memory_keep *keep = *pkeep;

	*pkeep = NULL;
	if (!keep || !refcount_dec_and_test(&keep->refcount))
		return;

	/* An IOVA is only parked with tags, which know their device. */
	if (keep->iova) {
		struct nvkm_device_tegra *tdev =
			nvkm_mem_tegra(keep->tags->device->mmu);

		mutex_lock(&tdev->iommu.mutex);
		nvkm_mm_free(&tdev->iommu.mm, &keep->iova);
		mutex_unlock(&tdev->iommu.mutex);
	}
	nvkm_memory_tags_put(&keep->tags);
	kfree(keep);
}

/* Flatten the scattered PAGE_SIZE pages into one contiguous IOVA range, so
 * the object can use the larger GPU page that compression requires.
 */
static int
nvkm_mem_iommu_init(struct nvkm_mem *mem, u8 page)
{
	struct nvkm_device_tegra *tdev = nvkm_mem_tegra(mem->mmu);
	struct nvkm_memory_keep *keep = mem->memory.keep;
	u64 addr, i, n;
	u32 align;
	int ret;

	if (!tdev || page >= tdev->func->iommu_bit)
		return -EOPNOTSUPP;

	/* A comptag line covers 128KiB; its halves follow IOVA bit 16. */
	align = max_t(u32, 1 << (page - PAGE_SHIFT), SZ_128K >> PAGE_SHIFT);

	mutex_lock(&tdev->iommu.mutex);
	if (keep && keep->iova) {
		ret = keep->iova->length == mem->pages ? 0 : -EINVAL;
		if (ret)
			goto unlock;

		/* An older memory object of the buffer may still translate it. */
		if (keep->user) {
			addr = (u64)keep->iova->offset << tdev->iommu.pgshift;
			iommu_unmap(tdev->iommu.domain, addr,
				    mem->pages << PAGE_SHIFT);
		}
		mem->iommu = keep->iova;
	} else {
		ret = nvkm_mm_head(&tdev->iommu.mm, 0, 1, mem->pages, mem->pages,
				   align, &mem->iommu);
		if (ret)
			goto unlock;
		if (keep)
			keep->iova = mem->iommu;
	}
	if (keep)
		keep->user = mem;

	addr = (u64)mem->iommu->offset << tdev->iommu.pgshift;

	/* dma[] is used as a physical address, as on the existing host path. */
	for (i = 0; i < mem->pages; i += n) {
		for (n = 1; i + n < mem->pages; n++) {
			if (mem->dma[i + n] != mem->dma[i] + (n << PAGE_SHIFT))
				break;
		}

		ret = iommu_map(tdev->iommu.domain, addr + (i << PAGE_SHIFT),
				mem->dma[i], n << PAGE_SHIFT,
				IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			nvkm_mem_iommu_put(tdev, mem, i);
			goto unlock;
		}
	}
unlock:
	mutex_unlock(&tdev->iommu.mutex);
	if (ret)
		return ret;

	mem->node.offset = (addr >> NVKM_RAM_MM_SHIFT) |
			   BIT_ULL(tdev->func->iommu_bit - NVKM_RAM_MM_SHIFT);
	mem->node.length = mem->pages << (PAGE_SHIFT - NVKM_RAM_MM_SHIFT);
	mem->node.next = NULL;
	mem->page = page;
	return 0;
}

static void *
nvkm_mem_dtor(struct nvkm_memory *memory)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	if (mem->iommu)
		nvkm_mem_iommu_fini(mem);
	nvkm_mem_keep_put(&mem->memory.keep);
	if (mem->mem) {
		while (mem->pages--) {
			dma_unmap_page(mem->mmu->subdev.device->dev,
				       mem->dma[mem->pages], PAGE_SIZE,
				       DMA_BIDIRECTIONAL);
			__free_page(mem->mem[mem->pages]);
		}
		kvfree(mem->dma);
		kvfree(mem->mem);
	}
	return mem;
}

static const struct nvkm_memory_func
nvkm_mem_dma = {
	.dtor = nvkm_mem_dtor,
	.target = nvkm_mem_target,
	.page = nvkm_mem_page,
	.addr = nvkm_mem_addr,
	.size = nvkm_mem_size,
	.map = nvkm_mem_map_dma,
};

static int
nvkm_mem_map_sgl(struct nvkm_memory *memory, u64 offset, struct nvkm_vmm *vmm,
		 struct nvkm_vma *vma, void *argv, u32 argc)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	struct nvkm_vmm_map map = {
		.memory = &mem->memory,
		.offset = offset,
		.sgl = mem->sgl,
	};
	return nvkm_vmm_map(vmm, vma, argv, argc, &map);
}

static const struct nvkm_memory_func
nvkm_mem_sgl = {
	.dtor = nvkm_mem_dtor,
	.target = nvkm_mem_target,
	.page = nvkm_mem_page,
	.addr = nvkm_mem_addr,
	.size = nvkm_mem_size,
	.map = nvkm_mem_map_sgl,
};

static const struct nvkm_memory_func
nvkm_mem_iommu = {
	.dtor = nvkm_mem_dtor,
	.target = nvkm_mem_target,
	.page = nvkm_mem_page,
	.addr = nvkm_mem_addr,
	.size = nvkm_mem_size,
	.map = nvkm_mem_map_iommu,
};

int
nvkm_mem_map_host(struct nvkm_memory *memory, void **pmap)
{
	struct nvkm_mem *mem = nvkm_mem(memory);
	if (mem->mem) {
		*pmap = vmap(mem->mem, mem->pages, VM_MAP, PAGE_KERNEL);
		return *pmap ? 0 : -EFAULT;
	}
	return -EINVAL;
}

static int
nvkm_mem_new_host(struct nvkm_mmu *mmu, int type, u8 page, u64 size,
		  void *argv, u32 argc, struct nvkm_memory **pmemory)
{
	struct device *dev = mmu->subdev.device->dev;
	union {
		struct nvif_mem_ram_vn vn;
		struct nvif_mem_ram_v0 v0;
	} *args = argv;
	int ret = -ENOSYS;
	enum nvkm_memory_target target;
	struct nvkm_mem *mem;
	gfp_t gfp = GFP_USER | __GFP_ZERO;

	if ( (mmu->type[type].type & NVKM_MEM_COHERENT) &&
	    !(mmu->type[type].type & NVKM_MEM_UNCACHED))
		target = NVKM_MEM_TARGET_HOST;
	else
		target = NVKM_MEM_TARGET_NCOH;

	if (page < PAGE_SHIFT)
		return -EINVAL;

	if (!(mem = kzalloc_obj(*mem)))
		return -ENOMEM;
	mem->target = target;
	mem->mmu = mmu;
	mem->page = PAGE_SHIFT;
	*pmemory = &mem->memory;

	if (!(ret = nvif_unpack(ret, &argv, &argc, args->v0, 0, 0, false))) {
		if (args->v0.dma) {
			nvkm_memory_ctor(&nvkm_mem_dma, &mem->memory);
			mem->dma = args->v0.dma;
		} else {
			nvkm_memory_ctor(&nvkm_mem_sgl, &mem->memory);
			mem->sgl = args->v0.sgl;
		}
		mem->memory.keep = args->v0.keep;
		if (mem->memory.keep)
			refcount_inc(&mem->memory.keep->refcount);

		if (!IS_ALIGNED(size, PAGE_SIZE))
			return -EINVAL;
		mem->pages = size >> PAGE_SHIFT;

		if (page > PAGE_SHIFT) {
			if (!args->v0.dma)
				return -EINVAL;
			ret = nvkm_mem_iommu_init(mem, page);
			if (ret)
				return ret;
			mem->memory.func = &nvkm_mem_iommu;
		}
		return 0;
	} else
	if ( (ret = nvif_unvers(ret, &argv, &argc, args->vn))) {
		kfree(mem);
		return ret;
	}

	nvkm_memory_ctor(&nvkm_mem_dma, &mem->memory);
	size = ALIGN(size, PAGE_SIZE) >> PAGE_SHIFT;

	if (!(mem->mem = kvmalloc_objs(*mem->mem, size)))
		return -ENOMEM;
	if (!(mem->dma = kvmalloc_objs(*mem->dma, size)))
		return -ENOMEM;

	if (mmu->dma_bits > 32)
		gfp |= GFP_HIGHUSER;
	else
		gfp |= GFP_DMA32;

	for (mem->pages = 0; size; size--, mem->pages++) {
		struct page *p = alloc_page(gfp);
		if (!p)
			return -ENOMEM;

		mem->dma[mem->pages] = dma_map_page(mmu->subdev.device->dev,
						    p, 0, PAGE_SIZE,
						    DMA_BIDIRECTIONAL);
		if (dma_mapping_error(dev, mem->dma[mem->pages])) {
			__free_page(p);
			return -ENOMEM;
		}

		mem->mem[mem->pages] = p;
	}

	return 0;
}

int
nvkm_mem_new_type(struct nvkm_mmu *mmu, int type, u8 page, u64 size,
		  void *argv, u32 argc, struct nvkm_memory **pmemory)
{
	struct nvkm_memory *memory = NULL;
	int ret;

	if (mmu->type[type].type & NVKM_MEM_VRAM) {
		ret = mmu->func->mem.vram(mmu, type, page, size,
					  argv, argc, &memory);
	} else {
		ret = nvkm_mem_new_host(mmu, type, page, size,
					argv, argc, &memory);
	}

	if (ret)
		nvkm_memory_unref(&memory);
	*pmemory = memory;
	return ret;
}
