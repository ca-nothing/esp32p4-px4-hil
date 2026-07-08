#pragma once
/*
 * fc_px4_netif_compat.h — Linux-compatible <net/if.h> for mavlink UDP transport
 * (force-include, first in every mavlink TU).
 *
 * mavlink_main.cpp find_broadcast_address() uses BSD interface enumeration (ifconf +
 * SIOCGIFCONF + ifreq.ifr_addr). lwIP's ifreq is minimal (only ifr_name), unguarded,
 * and used internally, so it can't be overridden. Fix: shadow lwIP's ifreq via rename,
 * then define the full Linux-compatible ifreq/ifconf/SIOC*.
 *
 * This header must pull in lwIP first (before the socket/poll force-includes) so the
 * rename catches lwIP's ifreq; later sys/socket.h includes skip via guards.
 *
 * Runtime: lwIP rejects SIOCGIFCONF, so the first ioctl fails and find_broadcast_address
 * returns early — the layout difference is never exercised. mavlink then unicasts to its
 * configured/learned remote (HIL / QGC). lwIP-core and PX4 stay read-only.
 * ---
 * fc_px4_netif_compat.h — mavlink UDP tasimasi icin Linux-uyumlu <net/if.h>
 * (force-include, her mavlink TU'sunda ilk sirada).
 *
 * mavlink_main.cpp find_broadcast_address() BSD arayuz numaralandirmasi kullanir (ifconf +
 * SIOCGIFCONF + ifreq.ifr_addr). lwIP'nin ifreq'i asgaridir (yalnizca ifr_name), korumasizdir
 * ve dahili kullanilir, bu yuzden gecersiz-kilinamaz. Cozum: lwIP'nin ifreq'ini yeniden-adlandirma
 * ile golgele, sonra tam Linux-uyumlu ifreq/ifconf/SIOC*'i tanimla.
 *
 * Bu baslik lwIP'yi once cekmeli (socket/poll force-include'lardan once) ki yeniden-adlandirma
 * lwIP'nin ifreq'ini yakalasin; sonraki sys/socket.h include'lari korumalarla atlanir.
 *
 * Calisma-zamani: lwIP SIOCGIFCONF'u reddeder, boylece ilk ioctl basarisiz olur ve
 * find_broadcast_address erken doner — yerlesim farki hic isletilmez. mavlink sonra
 * yapilandirilmis/ogrenilmis uzak-uca unicast yapar (HIL / QGC). lwIP-cekirdegi ve PX4 salt-okunur kalir.
 */

/* 1) Shadow lwIP's unguarded minimal ifreq under a different name:
   | 1) lwIP'nin korumasiz asgari ifreq'ini farkli bir ad altinda golgele: */
#define ifreq __lwip_shadow_ifreq
#include <sys/socket.h>    /* -> lwip/sockets.h: struct __lwip_shadow_ifreq{ifr_name} + struct sockaddr + _IO */
#undef ifreq

#ifndef IFNAMSIZ
#define IFNAMSIZ 16
#endif
#ifndef IF_NAMESIZE
#define IF_NAMESIZE IFNAMSIZ
#endif

/* 2) Full Linux-compatible ifreq (PX4 uses ifr_addr/ifr_name/ifr_netmask):
   | 2) Tam Linux-uyumlu ifreq (PX4 ifr_addr/ifr_name/ifr_netmask kullanir): */
struct ifreq {
	char ifr_name[IFNAMSIZ];
	union {
		struct sockaddr ifru_addr;
		struct sockaddr ifru_netmask;
		short           ifru_flags;
		int             ifru_ivalue;
	} ifr_ifru;
};
#define ifr_addr     ifr_ifru.ifru_addr
#define ifr_netmask  ifr_ifru.ifru_netmask
#define ifr_flags    ifr_ifru.ifru_flags

/* 3) ifconf (absent in lwIP): | 3) ifconf (lwIP'de yok): */
struct ifconf {
	int ifc_len;
	union {
		char         *ifcu_buf;
		struct ifreq *ifcu_req;
	} ifc_ifcu;
};
#define ifc_buf  ifc_ifcu.ifcu_buf
#define ifc_req  ifc_ifcu.ifcu_req

/* 4) SIOC* (not in lwIP; ioctl rejects these at runtime -> early return):
   | 4) SIOC* (lwIP'de yok; ioctl bunlari calisma-zamaninda reddeder -> erken donus): */
#ifndef SIOCGIFCONF
#define SIOCGIFCONF    0x8912
#endif
#ifndef SIOCGIFNETMASK
#define SIOCGIFNETMASK 0x891b
#endif
