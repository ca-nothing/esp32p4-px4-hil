# ESP32-C6 Companion Firmware

**🌐 Language / Dil:** [English](#english) · [Türkçe](#türkçe)

---

<a name="english"></a>
# English

`network_adapter.bin` — prebuilt **esp_hosted slave** firmware (~1.4 MB) that turns the
ESP32-C6 into the P4's Wi-Fi/BLE network co-processor over SDIO.

- **Target:** esp32c6 (built from Espressif's esp_hosted example; full source not included here).
- The ESP32-C6 already ships with **equivalent factory esp_hosted firmware** and works out of the box — the P4 resets and starts it over SDIO — so you normally do **not** need to flash this. It is included **for reference / reflash-if-needed** only.
- To reflash, flash it directly to the ESP32-C6 over the C6's own USB with the standard esptool flow for an `esp32c6` target. (The P4's partition table reserves no slot for it.)

---

<a name="türkçe"></a>
# Türkçe

`network_adapter.bin` — SDIO üzerinden ESP32-C6'yı P4'ün Wi-Fi/BLE ağ yardımcı-işlemcisine dönüştüren,
önceden derlenmiş **esp_hosted slave** firmware'i (~1.4 MB).

- **Hedef:** esp32c6 (Espressif'in esp_hosted örneğinden derlendi; tam kaynak burada bulunmaz).
- ESP32-C6, **eşdeğer fabrika esp_hosted firmware'i** ile gelir ve kutudan çıktığı gibi çalışır — P4 onu reset'leyip SDIO üzerinden başlatır — bu yüzden normalde bunu flash'lamanız **gerekmez**. Sadece **referans / gerekirse-yeniden-flash** için dahil edilmiştir.
- Yeniden flash'lamak için, C6'nın kendi USB'si üzerinden `esp32c6` hedefi için standart esptool akışıyla doğrudan ESP32-C6'ya flash'layın. (P4'ün partition tablosu bunun için yer ayırmaz.)
