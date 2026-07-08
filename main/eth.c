#include "eth.h"
#include "driver/gpio.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "FC_ETH";
static const char *NVS_NAMESPACE = "eth_cfg";
static const char *NVS_KEY_DHCP = "dhcp";
static const char *NVS_KEY_IP = "ip";
static const char *NVS_KEY_NETMASK = "nm";
static const char *NVS_KEY_GW = "gw";
static const char *NVS_KEY_DNS = "dns";
static const char *NVS_KEY_DNS2 = "dns2";

// Static handles for use by helper functions | Yardimci fonksiyonlarin kullanimi icin statik tutamaclar
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;

/**
 * ESP32-P4-NANO Ethernet (RMII) pinout:
 *
 * ESP32-P4 internal EMAC RMII pins (fixed by silicon):
 *   TX_EN   = GPIO49
 *   TXD0    = GPIO34
 *   TXD1    = GPIO35
 *   CRS_DV  = GPIO28
 *   RXD0    = GPIO29
 *   RXD1    = GPIO30
 *   RMII_CLK = GPIO50 (external 50MHz oscillator)
 *
 * SMI (MDC/MDIO) - configurable:
 *   MDC     = GPIO31 (default)
 *   MDIO    = GPIO52 (default)
 *
 * IP101GRI PHY:
 *   PHY_ADDR = 1 (default)
 *   Reset    = GPIO51
 * ---
 * ESP32-P4-NANO Ethernet (RMII) pin dizilimi:
 *
 * ESP32-P4 dahili EMAC RMII pinleri (silikonla sabit):
 *   TX_EN   = GPIO49
 *   TXD0    = GPIO34
 *   TXD1    = GPIO35
 *   CRS_DV  = GPIO28
 *   RXD0    = GPIO29
 *   RXD1    = GPIO30
 *   RMII_CLK = GPIO50 (harici 50MHz osilator)
 *
 * SMI (MDC/MDIO) - yapilandirilabilir:
 *   MDC     = GPIO31 (varsayilan)
 *   MDIO    = GPIO52 (varsayilan)
 *
 * IP101GRI PHY:
 *   PHY_ADDR = 1 (varsayilan)
 *   Reset    = GPIO51
 */

esp_err_t eth_init(void) {
  ESP_LOGI(TAG, "Starting Ethernet (RMII)...");

  // Create default Ethernet network interface | Varsayilan Ethernet ag arayuzunu olustur
  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
  if (!eth_netif) {
    ESP_LOGE(TAG, "esp_netif_new failed");
    return ESP_FAIL;
  }

  // ESP32-P4 specific EMAC configuration | ESP32-P4'e ozgu EMAC yapilandirmasi
  eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  esp32_emac_config.smi_gpio.mdc_num = 31;  // MDC GPIO
  esp32_emac_config.smi_gpio.mdio_num = 52; // MDIO GPIO

  // Ethernet MAC configuration | Ethernet MAC yapilandirmasi
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();

  // Create ESP32-P4 EMAC MAC instance | ESP32-P4 EMAC MAC ornegini olustur
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
  if (!mac) {
    ESP_LOGE(TAG, "MAC creation failed");
    esp_netif_destroy(eth_netif);
    return ESP_FAIL;
  }

  // Ethernet PHY configuration - IP101GRI | Ethernet PHY yapilandirmasi - IP101GRI
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = 51;
  phy_config.autonego_timeout_ms = 5000;
  phy_config.hw_reset_assert_time_us = 10000;
  phy_config.post_hw_reset_delay_ms = 100;
  phy_config.reset_timeout_ms = 100;

  esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
  if (!phy) {
    ESP_LOGE(TAG, "PHY creation failed");
    mac->del(mac);
    esp_netif_destroy(eth_netif);
    return ESP_FAIL;
  }

  // Ethernet driver configuration | Ethernet surucu yapilandirmasi
  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;

  esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(ret));
    mac->del(mac);
    phy->del(phy);
    esp_netif_destroy(eth_netif);
    return ret;
  }

  // Attach Ethernet driver to netif | Ethernet surucusunu netif'e bagla
  esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
  if (!glue) {
    ESP_LOGE(TAG, "netif glue creation failed");
    esp_eth_driver_uninstall(eth_handle);
    esp_netif_destroy(eth_netif);
    return ESP_FAIL;
  }

  ret = esp_netif_attach(eth_netif, glue);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "netif attach failed: %s", esp_err_to_name(ret));
    esp_eth_driver_uninstall(eth_handle);
    esp_netif_destroy(eth_netif);
    return ret;
  }

  // Enable TX/RX traffic event reporting | TX/RX trafik olay raporlamasini etkinlestir
  esp_netif_tx_rx_event_enable(eth_netif);

  // Start Ethernet driver state machine | Ethernet surucu durum-makinesini baslat
  ret = esp_eth_start(eth_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Ethernet start failed: %s", esp_err_to_name(ret));
    esp_eth_del_netif_glue(glue);
    esp_eth_driver_uninstall(eth_handle);
    esp_netif_destroy(eth_netif);
    return ret;
  }

  // Store handles for helper functions | Tutamaclari yardimci fonksiyonlar icin sakla
  s_eth_netif = eth_netif;
  s_eth_handle = eth_handle;

  ESP_LOGI(TAG, "Ethernet attached. Acquiring IP via DHCP...");
  return ESP_OK;
}

// ---- NVS PERSISTENCE | NVS KALICILIK ----

static esp_err_t eth_save_config_to_nvs(void) {
  if (!s_eth_netif)
    return ESP_ERR_INVALID_STATE;

  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(ret));
    return ret;
  }

  bool dhcp = eth_is_dhcp();
  nvs_set_u8(nvs, NVS_KEY_DHCP, dhcp ? 1 : 0);

  esp_netif_ip_info_t ip_info;
  esp_netif_dns_info_t dns_info;
  if (esp_netif_get_ip_info(s_eth_netif, &ip_info) == ESP_OK) {
    nvs_set_u32(nvs, NVS_KEY_IP, ip_info.ip.addr);
    nvs_set_u32(nvs, NVS_KEY_NETMASK, ip_info.netmask.addr);
    nvs_set_u32(nvs, NVS_KEY_GW, ip_info.gw.addr);
  }

  if (esp_netif_get_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns_info) ==
      ESP_OK) {
    nvs_set_u32(nvs, NVS_KEY_DNS, dns_info.ip.u_addr.ip4.addr);
  }
  if (esp_netif_get_dns_info(s_eth_netif, ESP_NETIF_DNS_BACKUP, &dns_info) ==
      ESP_OK) {
    nvs_set_u32(nvs, NVS_KEY_DNS2, dns_info.ip.u_addr.ip4.addr);
  }

  nvs_commit(nvs);
  nvs_close(nvs);

  ESP_LOGI(TAG, "Eth config saved to NVS (DHCP=%s)", dhcp ? "yes" : "no");
  return ESP_OK;
}

void eth_load_config_from_nvs(void) {
  if (!s_eth_netif)
    return;

  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
    ESP_LOGI(TAG, "No saved Ethernet config in NVS, using DHCP");
    return;
  }

  uint8_t dhcp_val = 1;
  nvs_get_u8(nvs, NVS_KEY_DHCP, &dhcp_val);

  if (dhcp_val == 0) {
    esp_netif_ip_info_t ip_info;
    esp_netif_dns_info_t dns_info, dns2_info;
    uint32_t ip_u = 0, nm_u = 0, gw_u = 0, dns_u = 0, dns2_u = 0;

    nvs_get_u32(nvs, NVS_KEY_IP, &ip_u);
    nvs_get_u32(nvs, NVS_KEY_NETMASK, &nm_u);
    nvs_get_u32(nvs, NVS_KEY_GW, &gw_u);
    nvs_get_u32(nvs, NVS_KEY_DNS, &dns_u);
    nvs_get_u32(nvs, NVS_KEY_DNS2, &dns2_u);

    nvs_close(nvs);

    if (ip_u == 0) {
      ESP_LOGI(TAG, "No valid static IP in NVS, using DHCP");
      return;
    }

    memset(&ip_info, 0, sizeof(ip_info));
    memset(&dns_info, 0, sizeof(dns_info));
    memset(&dns2_info, 0, sizeof(dns2_info));
    ip_info.ip.addr = ip_u;
    ip_info.netmask.addr = nm_u;
    ip_info.gw.addr = gw_u;
    dns_info.ip.u_addr.ip4.addr = dns_u;
    dns2_info.ip.u_addr.ip4.addr = dns2_u;

    char ipstr[16];
    esp_ip4addr_ntoa(&ip_info.ip, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "Loading static config from NVS: IP=%s", ipstr);

    esp_netif_dhcpc_stop(s_eth_netif);
    esp_netif_set_ip_info(s_eth_netif, &ip_info);
    if (dns_u != 0) {
      esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns_info);
    }
    if (dns2_u != 0) {
      esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_BACKUP, &dns2_info);
    }
  } else {
    nvs_close(nvs);
    ESP_LOGI(TAG, "Loading DHCP config from NVS");
  }
}

// ---- HELPER FUNCTIONS | YARDIMCI FONKSIYONLAR ----

esp_netif_t *eth_get_netif(void) { return s_eth_netif; }

esp_eth_handle_t eth_get_handle(void) { return s_eth_handle; }

esp_err_t eth_get_mac(uint8_t *mac) {
  if (!s_eth_handle || !mac)
    return ESP_ERR_INVALID_ARG;
  return esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac);
}

esp_err_t eth_get_ip_config(esp_netif_ip_info_t *ip_info,
                            esp_netif_dns_info_t *dns) {
  if (!s_eth_netif || !ip_info || !dns)
    return ESP_ERR_INVALID_ARG;

  esp_err_t ret = esp_netif_get_ip_info(s_eth_netif, ip_info);
  if (ret != ESP_OK)
    return ret;

  ret = esp_netif_get_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, dns);
  return ret;
}

esp_err_t eth_set_static_ip(const esp_netif_ip_info_t *ip_info,
                            const esp_netif_dns_info_t *dns_primary,
                            const esp_netif_dns_info_t *dns_secondary) {
  if (!s_eth_netif)
    return ESP_ERR_INVALID_STATE;

  // Stop DHCP client first | Once DHCP istemcisini durdur
  esp_err_t ret = esp_netif_dhcpc_stop(s_eth_netif);
  if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
    ESP_LOGW(TAG, "DHCP stop failed: %s", esp_err_to_name(ret));
  }

  // Set static IP | Statik IP ayarla
  ret = esp_netif_set_ip_info(s_eth_netif, ip_info);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Static IP set failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Set primary DNS | Birincil DNS ayarla
  if (dns_primary && dns_primary->ip.u_addr.ip4.addr != 0) {
    ret = esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN,
                                 (esp_netif_dns_info_t *)dns_primary);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Primary DNS set failed: %s", esp_err_to_name(ret));
    }
  }

  // Set secondary DNS | Ikincil DNS ayarla
  if (dns_secondary && dns_secondary->ip.u_addr.ip4.addr != 0) {
    ret = esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_BACKUP,
                                 (esp_netif_dns_info_t *)dns_secondary);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Secondary DNS set failed: %s", esp_err_to_name(ret));
    }
  }

  // Save to NVS | NVS'e kaydet
  eth_save_config_to_nvs();

  ESP_LOGI(TAG, "Static IP configured");
  return ESP_OK;
}

esp_err_t eth_enable_dhcp(void) {
  if (!s_eth_netif)
    return ESP_ERR_INVALID_STATE;

  esp_err_t ret = esp_netif_dhcpc_start(s_eth_netif);
  if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
    ESP_LOGE(TAG, "DHCP start failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Save to NVS | NVS'e kaydet
  eth_save_config_to_nvs();

  ESP_LOGI(TAG, "DHCP enabled");
  return ESP_OK;
}

bool eth_is_dhcp(void) {
  if (!s_eth_netif)
    return false;
  esp_netif_dhcp_status_t status;
  if (esp_netif_dhcpc_get_status(s_eth_netif, &status) == ESP_OK) {
    return (status == ESP_NETIF_DHCP_STARTED);
  }
  return false;
}

esp_err_t eth_get_link_info(int *speed_mbps, bool *full_duplex) {
  if (!s_eth_handle)
    return ESP_ERR_INVALID_STATE;

  eth_speed_t speed;
  eth_duplex_t duplex;

  esp_err_t ret = esp_eth_ioctl(s_eth_handle, ETH_CMD_G_SPEED, &speed);
  if (ret != ESP_OK)
    return ret;

  ret = esp_eth_ioctl(s_eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex);
  if (ret != ESP_OK)
    return ret;

  if (speed_mbps) {
    switch (speed) {
    case ETH_SPEED_10M:
      *speed_mbps = 10;
      break;
    case ETH_SPEED_100M:
      *speed_mbps = 100;
      break;
    default:
      *speed_mbps = 0;
      break;
    }
  }

  if (full_duplex)
    *full_duplex = (duplex == ETH_DUPLEX_FULL);

  return ESP_OK;
}

esp_err_t eth_save_config(void) { return eth_save_config_to_nvs(); }
