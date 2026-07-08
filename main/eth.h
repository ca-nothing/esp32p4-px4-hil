#pragma once
#include "esp_eth.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file eth.h
 * @brief Ethernet (RMII) driver for ESP32-P4-NANO
 *
 * Uses ESP32-P4 internal EMAC with IP101GRI PHY.
 * Supports DHCP (default) and static IP with NVS persistence.
 *
 * Pinout (RMII):
 *   TX_EN=49, TXD0=34, TXD1=35, CRS_DV=28,
 *   RXD0=29, RXD1=30, RMII_CLK=50 (ext osc)
 *   MDC=31, MDIO=52, PHY_ADDR=1, Reset=51
 * ---
 * @brief ESP32-P4-NANO icin Ethernet (RMII) surucusu
 *
 * IP101GRI PHY ile ESP32-P4 dahili EMAC kullanir.
 * DHCP (varsayilan) ve NVS-kalicilikli statik IP destekler.
 *
 * Pin dizilimi (RMII):
 *   TX_EN=49, TXD0=34, TXD1=35, CRS_DV=28,
 *   RXD0=29, RXD1=30, RMII_CLK=50 (harici osilator)
 *   MDC=31, MDIO=52, PHY_ADDR=1, Reset=51
 */

esp_err_t eth_init(void);
esp_netif_t *eth_get_netif(void);
esp_eth_handle_t eth_get_handle(void);
esp_err_t eth_get_mac(uint8_t *mac);
esp_err_t eth_get_ip_config(esp_netif_ip_info_t *ip_info,
                            esp_netif_dns_info_t *dns);
esp_err_t eth_set_static_ip(const esp_netif_ip_info_t *ip_info,
                            const esp_netif_dns_info_t *dns_primary,
                            const esp_netif_dns_info_t *dns_secondary);
esp_err_t eth_enable_dhcp(void);
bool eth_is_dhcp(void);
esp_err_t eth_get_link_info(int *speed_mbps, bool *full_duplex);

/**
 * @brief Save current Ethernet config to NVS (DHCP or static)
 * ---
 * @brief Gecerli Ethernet yapilandirmasini NVS'e kaydet (DHCP veya statik)
 */
esp_err_t eth_save_config(void);

/**
 * @brief Load saved Ethernet config from NVS and apply
 * ---
 * @brief Kayitli Ethernet yapilandirmasini NVS'ten yukle ve uygula
 */
void eth_load_config_from_nvs(void);

#ifdef __cplusplus
}
#endif
