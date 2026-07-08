/*
 * fc_internal_new.cpp — FC_INTERNAL_NEW: internal-SRAM-first allocator with PSRAM fallback.
 *
 * ESP-IDF default (CONFIG_SPIRAM_USE_MALLOC + ALWAYSINTERNAL=2048) puts >2KB allocations in
 * slow PSRAM, so large objects (EKF2's 24x24 covariance + buffers, SimulatorMavlink, big uORB
 * objects) land there and every 250Hz access is ~10x slower (wq:INS0 ~1240us vs ~174us algorithm).
 *
 * Override global operator new to try fast internal SRAM first, falling back to PSRAM when full
 * (no OOM). delete = heap_caps_free, which resolves any heap pointer, so it stays default-new compatible.
 *
 * Platform layer only — does not touch PX4 code. FC_INTERNAL_NEW=0 -> no override (faithful).
 * ---
 * fc_internal_new.cpp — FC_INTERNAL_NEW: PSRAM'e geri-düşmeli, önce-iç-SRAM ayırıcı.
 *
 * ESP-IDF varsayılanı (CONFIG_SPIRAM_USE_MALLOC + ALWAYSINTERNAL=2048) >2KB ayırmaları yavaş
 * PSRAM'e koyar, bu yüzden büyük nesneler (EKF2'nin 24x24 kovaryansı + tamponları, SimulatorMavlink,
 * büyük uORB nesneleri) oraya düşer ve her 250Hz erişim ~10x daha yavaştır (wq:INS0 ~1240us,
 * algoritmanın ~174us'sine karşı).
 *
 * Global operator new'i önce hızlı iç SRAM'i deneyecek, dolunca PSRAM'e geri düşecek (OOM yok) şekilde
 * geçersiz kıl. delete = heap_caps_free, herhangi bir heap işaretçisini çözer, bu yüzden varsayılan-new
 * ile uyumlu kalır.
 *
 * Yalnızca platform katmanı — PX4 koduna dokunmaz. FC_INTERNAL_NEW=0 -> geçersiz kılma yok (sadık).
 */
#if FC_INTERNAL_NEW

#include <cstddef>
#include <cstdlib>
#include <new>
#include "esp_heap_caps.h"

static inline void *fc_inew(std::size_t sz)
{
	void *p = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (!p) { p = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
	if (!p) { p = malloc(sz); }
	return p;
}

static inline void *fc_inew_aligned(std::size_t sz, std::size_t al)
{
	void *p = heap_caps_aligned_alloc(al, sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (!p) { p = heap_caps_aligned_alloc(al, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
	return p;
}

/* --- throwing new (PX4 -fno-exceptions: null return matches default behavior) ---
   | --- throwing new (PX4 -fno-exceptions: null dönüş varsayılan davranışla eşleşir) --- */
void *operator new(std::size_t sz)   { return fc_inew(sz); }
void *operator new[](std::size_t sz) { return fc_inew(sz); }

/* --- nothrow new | --- nothrow (istisna-atmayan) new --- */
void *operator new(std::size_t sz, const std::nothrow_t &) noexcept   { return fc_inew(sz); }
void *operator new[](std::size_t sz, const std::nothrow_t &) noexcept { return fc_inew(sz); }

/* --- aligned new (C++17) | --- hizalı (aligned) new (C++17) --- */
void *operator new(std::size_t sz, std::align_val_t al)   { return fc_inew_aligned(sz, static_cast<std::size_t>(al)); }
void *operator new[](std::size_t sz, std::align_val_t al) { return fc_inew_aligned(sz, static_cast<std::size_t>(al)); }
void *operator new(std::size_t sz, std::align_val_t al, const std::nothrow_t &) noexcept   { return fc_inew_aligned(sz, static_cast<std::size_t>(al)); }
void *operator new[](std::size_t sz, std::align_val_t al, const std::nothrow_t &) noexcept { return fc_inew_aligned(sz, static_cast<std::size_t>(al)); }

/* --- delete (heap_caps_free resolves any heap) | --- delete (heap_caps_free herhangi bir heap'i çözer) --- */
void operator delete(void *p) noexcept   { heap_caps_free(p); }
void operator delete[](void *p) noexcept { heap_caps_free(p); }
void operator delete(void *p, std::size_t) noexcept   { heap_caps_free(p); }
void operator delete[](void *p, std::size_t) noexcept { heap_caps_free(p); }
void operator delete(void *p, std::align_val_t) noexcept   { heap_caps_free(p); }
void operator delete[](void *p, std::align_val_t) noexcept { heap_caps_free(p); }
void operator delete(void *p, std::size_t, std::align_val_t) noexcept   { heap_caps_free(p); }
void operator delete[](void *p, std::size_t, std::align_val_t) noexcept { heap_caps_free(p); }
void operator delete(void *p, const std::nothrow_t &) noexcept   { heap_caps_free(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept { heap_caps_free(p); }

#endif /* FC_INTERNAL_NEW */
