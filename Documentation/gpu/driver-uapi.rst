===============
DRM Driver uAPI
===============

.. contents::

drm/i915 uAPI
=============

.. kernel-doc:: include/uapi/drm/i915_drm.h

drm/nouveau uAPI
================

VM_BIND / EXEC uAPI
-------------------

.. kernel-doc:: drivers/gpu/drm/nouveau/nouveau_exec.c
    :doc: Overview

.. kernel-doc:: include/uapi/drm/nouveau_drm.h

Buffer object CPU caching
-------------------------

``NOUVEAU_GEM_DOMAIN_COHERENT`` and ``NOUVEAU_GEM_DOMAIN_WC`` in the domain
of ``DRM_IOCTL_NOUVEAU_GEM_NEW`` affect how a buffer's system-memory backing
is mapped: GART, and system memory while the buffer is evicted. Without
either, that backing is mapped cached (write-combined behind an AGP bridge).
VRAM is mapped write-combined through the BAR in every case, and GPU-side
tiling, kinds and compression do not depend on these flags.

``NOUVEAU_GEM_DOMAIN_COHERENT``
  CPU and GPU accesses to the buffer are coherent, so the CPU can poll
  what the GPU writes with no ioctl in between. Where the GPU's accesses
  to system memory are coherent with the CPU caches, the buffer keeps a
  cached mapping. Where they are not, the kernel maps it write-combined
  instead and, where it lets the GPU's L2 cache system memory, keeps the
  buffer out of the L2.

``NOUVEAU_GEM_DOMAIN_WC`` (driver version 1.4.5)
  The CPU mapping is write-combined; nothing else changes. This is not
  coherence: CPU access still has to be bracketed by
  ``DRM_IOCTL_NOUVEAU_GEM_CPU_PREP`` and ``DRM_IOCTL_NOUVEAU_GEM_CPU_FINI``
  as for a cached buffer, and the kernel keeps maintaining the GPU L2
  around submissions. Together with COHERENT, the buffer has COHERENT's
  guarantee and a write-combined CPU mapping, also where COHERENT alone
  would keep it cached. Earlier versions ignore the flag and return a
  cached buffer, and ``DRM_IOCTL_NOUVEAU_GEM_INFO`` does not report it, so
  the driver version is the way to tell.

A buffer exported as a dma-buf keeps its CPU caching in the exporter's
mappings, the dma-buf mmap and vmap. The scatter-gather table an importer
receives carries no cache policy: an importer that maps those pages by
other means is responsible for their cache maintenance.

drm/panthor uAPI
================

.. kernel-doc:: include/uapi/drm/panthor_drm.h

drm/xe uAPI
===========

.. kernel-doc:: include/uapi/drm/xe_drm.h

drm/asahi uAPI
================

.. kernel-doc:: include/uapi/drm/asahi_drm.h
