/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#ifndef __IOT_INFRA_H__
#define __IOT_INFRA_H__
#if defined(__cplusplus)
extern "C" {
#endif

#define _PLATFORM_IS_HOST_

#define WIFI_PROVISION_ENABLED
#define AWSS_SUPPORT_DEV_AP
#define AWSS_SUPPORT_SMARTCONFIG
//#define AWSS_SUPPORT_SMARTCONFIG_MCAST
#define AWSS_SUPPORT_SMARTCONFIG_WPS
#define AWSS_SUPPORT_ZEROCONFIG

#ifndef AWSS_SUPPORT_ZEROCONFIG
#define AWSS_DISABLE_ENROLLEE
#define AWSS_DISABLE_REGISTRAR
#endif


#define ALCS_ENABLED
#define	ALCS_SERVER_ENABLED


#define AWSS_SUPPORT_STATIS
#define AWSS_SUPPORT_DEV_BIND_STATIS
#define DEV_BIND_ENABLED
#define DEVICE_MODEL_ENABLED
#define LOG_REPORT_TO_CLOUD
#define AWSS_SUPPORT_APLIST
#define FORCE_SSL_VERIFY
#define MQTT_COMM_ENABLED
#define MQTT_DIRECT
#define OTA_ENABLED
#define SUPPORT_TLS
#define COAP_SERV_MULTITHREAD
#define MQTT_PREAUTH_SUPPORT_HTTPS_CDN
#define MQTT_AUTO_SUBSCRIBE



#define OTA_SIGNAL_CHANNEL  1
#define WITH_MQTT_ZIP_TOPIC 1
#define WITH_MQTT_SUB_SHORTCUT 1 
#define WITH_MQTT_DYN_BUF 0

#ifndef CONFIG_HTTP_AUTH_TIMEOUT
#define CONFIG_HTTP_AUTH_TIMEOUT 5000
#endif
#ifndef CONFIG_MID_HTTP_TIMEOUT
#define CONFIG_MID_HTTP_TIMEOUT  5000
#endif
#ifndef CONFIG_GUIDER_AUTH_TIMEOUT
#define CONFIG_GUIDER_AUTH_TIMEOUT 10000
#endif





#if defined(__cplusplus)
}
#endif
#endif  /* __IOT_INFRA_H__ */
