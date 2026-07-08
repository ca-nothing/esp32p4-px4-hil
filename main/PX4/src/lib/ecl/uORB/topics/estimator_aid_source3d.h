#pragma once
/* FIX 6, Asama 2 - PX4 ECL portu icin uyumluluk header'i.
 * Bu dosya PX4-Autopilot kaynagindan kopyalanmadi: estimator_aid_source*d.h
 * dosyalari PX4 build sirasinda msg/EstimatorAidSource3d.msg dosyasindan
 * Tools/msg/templates/uorb/msg.h.em sablonuyla otomatik uretilir (statik
 * kaynak olarak repoda yoktur). EKF cekirdegi bu struct'i sadece duz uye
 * degisken olarak kullandigindan (orb_advertise/publish/subscribe API'si
 * cagrilmadigindan), uORB pub/sub altyapisi (ORB_DECLARE, __EXPORT) burada
 * kullanilmiyor. Alanlar msg/EstimatorAidSource3d.msg ile birebir aynidir.
 */

#include <cstdint>

struct estimator_aid_source3d_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;

	uint8_t estimator_instance;

	uint32_t device_id;

	uint64_t time_last_fuse;

	float observation[3];
	float observation_variance[3];

	float innovation[3];
	float innovation_filtered[3];

	float innovation_variance[3];

	float test_ratio[3];
	float test_ratio_filtered[3];

	bool innovation_rejected;
	bool fused;
};
