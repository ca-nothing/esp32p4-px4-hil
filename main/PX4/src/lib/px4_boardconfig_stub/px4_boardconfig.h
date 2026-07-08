#pragma once
/* FIX 6, Asama 2 - PX4 ECL portu icin uyumluluk stub'i.
 * Gercek PX4-Autopilot'ta bu header NuttX board konfigurasyonunu (CONFIG_*)
 * saglar. P4/ESP-IDF derlemesinde __PX4_NUTTX/__PX4_POSIX tanimli olmadigi
 * icin px4_platform_common/defines.h bu dosyadaki hicbir makroyu kullanmaz;
 * sadece #include satirinin derlenebilmesi icin bos olarak saglanir.
 */
