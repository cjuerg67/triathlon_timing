#pragma once

#include <cstddef>

#include "esp_err.h"

namespace runtime_settings {

esp_err_t init();

void getLanBrokerUri(char *out, size_t out_size);
void getGprsBrokerUri(char *out, size_t out_size);

void getBrokerIps(char *lan_ip_out, size_t lan_ip_out_size,
                  char *gprs_ip_out, size_t gprs_ip_out_size);

esp_err_t setBrokerIps(const char *lan_ip, const char *gprs_ip);

}  // namespace runtime_settings
