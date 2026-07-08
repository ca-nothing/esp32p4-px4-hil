#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "FC_HTTP";
static httpd_handle_t server = NULL;
static esp_err_t static_file_handler(httpd_req_t *req) {
  char filepath[CONFIG_HTTPD_MAX_URI_LEN + 8];
  if (strcmp(req->uri, "/") == 0) {
    strcpy(filepath, "/littlefs/index.html");
  } else {
    int ret = snprintf(filepath, sizeof(filepath), "/littlefs%s", req->uri);
    if (ret < 0 || ret >= sizeof(filepath)) {
      ESP_LOGE(TAG, "URI too long: %s", req->uri);
      httpd_resp_send_500(req);
      return ESP_OK;
    }
  }

  FILE *f = fopen(filepath, "r");
  if (!f) {
    // Try index.html for any unknown path | Bilinmeyen herhangi bir yol icin index.html'i dene
    f = fopen("/littlefs/index.html", "r");
    if (!f) {
      ESP_LOGW(TAG, "File not found: %s (LittleFS mounted?)", req->uri);
      httpd_resp_set_status(req, "404 Not Found");
      httpd_resp_send(req, "Not Found", 9);
      return ESP_OK;
    }
  }

  // Determine content type | Icerik turunu belirle
  const char *ext = strrchr(filepath, '.');
  if (ext) {
    if (strcmp(ext, ".html") == 0)
      httpd_resp_set_type(req, "text/html; charset=utf-8");
    else if (strcmp(ext, ".css") == 0)
      httpd_resp_set_type(req, "text/css; charset=utf-8");
    else if (strcmp(ext, ".js") == 0)
      httpd_resp_set_type(req, "application/javascript");
    else if (strcmp(ext, ".png") == 0)
      httpd_resp_set_type(req, "image/png");
    else if (strcmp(ext, ".ico") == 0)
      httpd_resp_set_type(req, "image/x-icon");
    else if (strcmp(ext, ".json") == 0)
      httpd_resp_set_type(req, "application/json");
    else if (strcmp(ext, ".svg") == 0)
      httpd_resp_set_type(req, "image/svg+xml");
    else
      httpd_resp_set_type(req, "text/plain; charset=utf-8");
  } else {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
  }

  // Stream file content | Dosya icerigini akit
  char buf[512];
  size_t nread;
  while ((nread = fread(buf, 1, sizeof(buf), f)) > 0) {
    if (httpd_resp_send_chunk(req, buf, nread) != ESP_OK) {
      fclose(f);
      return ESP_FAIL;
    }
  }
  fclose(f);

  return httpd_resp_send_chunk(req, NULL, 0);
}

static const httpd_uri_t uri_table[] = {
    {"/", HTTP_GET, static_file_handler, NULL},
    {"/*", HTTP_GET, static_file_handler, NULL},
};

// ==================== INIT | BASLATMA ====================
esp_err_t http_server_init(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 4;
  config.stack_size = 4096;
  config.lru_purge_enable = true;
  config.core_id = 0; // HTTP on Core 0 (Core 1 is FC) | HTTP Core 0'da (Core 1 = FC)
  config.uri_match_fn = httpd_uri_match_wildcard;

  ESP_LOGI(TAG, "Starting HTTP server (port %d)...", config.server_port);
  esp_err_t ret = httpd_start(&server, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(ret));
    return ret;
  }

  for (int i = 0; i < sizeof(uri_table) / sizeof(uri_table[0]); i++) {
    httpd_register_uri_handler(server, &uri_table[i]);
  }

  ESP_LOGI(TAG, "HTTP server ready (static page only): http://0.0.0.0:80/");
  return ESP_OK;
}

esp_err_t http_server_stop(void) {
  if (server) {
    httpd_stop(server);
    server = NULL;
  }
  return ESP_OK;
}
