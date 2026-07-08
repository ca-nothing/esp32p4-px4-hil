#include "littlefs_fs.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include <dirent.h>

static const char *TAG = "FC_LFS";

esp_err_t littlefs_fs_init(void) {
  ESP_LOGI(TAG, "LittleFS mounting...");

  esp_vfs_littlefs_conf_t conf = {
      .base_path = LITTLEFS_MOUNT_POINT,
      .partition_label = LITTLEFS_PARTITION,
      .partition = NULL,
      .format_if_mount_failed = false,
      .read_only = false,
      .dont_mount = false,
      .grow_on_mount = false,
  };

  esp_err_t ret = esp_vfs_littlefs_register(&conf);
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(TAG, "LittleFS partition not found — image flashed?");
    } else {
      ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
    }
    return ret;
  }

  size_t total = 0, used = 0;
  ret = esp_littlefs_info(LITTLEFS_PARTITION, &total, &used);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "LittleFS mounted: %zu KB total, %zu KB used", total / 1024,
             used / 1024);
  }

  // List files | Dosyalari listele
  DIR *dir = opendir(LITTLEFS_MOUNT_POINT);
  if (dir) {
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
      ESP_LOGD(TAG, "  %s/%s", LITTLEFS_MOUNT_POINT, entry->d_name);
    }
    closedir(dir);
  }

  return ESP_OK;
}

esp_err_t littlefs_fs_get_info(size_t *total, size_t *used) {
  return esp_littlefs_info(LITTLEFS_PARTITION, total, used);
}
