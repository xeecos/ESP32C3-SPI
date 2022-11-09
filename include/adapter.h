// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

#define PRIO_Q_HIGH             0
#define PRIO_Q_MID              1
#define PRIO_Q_LOW              2
#define MAX_PRIORITY_QUEUES     3
#define MAC_ADDR_LEN			6
#define MAX_KEY_LEN             32
#define MAX_SEQ_LEN             10
#define ESP_MAX_KEY_INDEX       0

/* ESP Payload Header Flags */
#define MORE_FRAGMENT			(1 << 0)
#define MAX_SSID_LEN			32

struct esp_payload_header {
	uint8_t          if_type:4;
	uint8_t          if_num:4;
	uint8_t          flags;
	uint8_t			 packet_type;
	uint8_t          reserved1;
	uint16_t         len;
	uint16_t         offset;
	uint16_t         checksum;
	uint8_t          reserved2;
	/* Position of union field has to always be last,
	 * this is required for hci_pkt_type */
	union {
		uint8_t      reserved3;
		uint8_t      hci_pkt_type;		/* Packet type for HCI interface */
		uint8_t      priv_pkt_type;		/* Packet type for priv interface */
	};
	/* Do no add anything here */
} __attribute__((packed));

enum ESP_INTERFACE_TYPE{
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_HCI_IF,
	ESP_INTERNAL_IF,
	ESP_MAX_IF,
};

enum ESP_PACKET_TYPE {
	PACKET_TYPE_DATA,
	PACKET_TYPE_COMMAND_REQUEST,
	PACKET_TYPE_COMMAND_RESPONSE,
	PACKET_TYPE_EVENT,
	PACKET_TYPE_EAPOL,
};

enum ESP_HOST_INTERRUPT {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
};

enum ESP_CAPABILITIES {
	ESP_WLAN_SDIO_SUPPORT = (1 << 0),
	ESP_BT_UART_SUPPORT = (1 << 1),
	ESP_BT_SDIO_SUPPORT = (1 << 2),
	ESP_BLE_ONLY_SUPPORT = (1 << 3),
	ESP_BR_EDR_ONLY_SUPPORT = (1 << 4),
	ESP_WLAN_SPI_SUPPORT = (1 << 5),
	ESP_BT_SPI_SUPPORT = (1 << 6),
};

enum ESP_INTERNAL_MSG {
    ESP_INTERNAL_BOOTUP_EVENT = 1,
};

enum ESP_BOOTUP_TAG_TYPE {
	ESP_BOOTUP_CAPABILITY,
	ESP_BOOTUP_FW_DATA,
	ESP_BOOTUP_SPI_CLK_MHZ,
	ESP_BOOTUP_FIRMWARE_CHIP_ID,
};

enum COMMAND_CODE {
	CMD_INIT_INTERFACE = 1,
	CMD_SET_MAC,
	CMD_GET_MAC,
	CMD_SCAN_REQUEST,
	CMD_STA_CONNECT,
	CMD_STA_DISCONNECT,
	CMD_DEINIT_INTERFACE,
	CMD_ADD_KEY,
	CMD_DEL_KEY,
	CMD_SET_DEFAULT_KEY,
	CMD_MAX,
};

enum EVENT_CODE {
	EVENT_SCAN_RESULT = 1,
	EVENT_STA_CONNECT,
	EVENT_STA_DISCONNECT,
};

enum COMMAND_RESPONSE_TYPE {
	CMD_RESPONSE_PENDING,
	CMD_RESPONSE_FAIL,
	CMD_RESPONSE_SUCCESS,
	CMD_RESPONSE_BUSY,
	CMD_RESPONSE_UNSUPPORTED,
	CMD_RESPONSE_INVALID,
};

struct command_header {
	uint8_t cmd_code;
	uint8_t cmd_status;
	uint16_t len;
	uint16_t seq_num;
}__attribute__((packed));

struct scan_request {
	struct command_header header;
	char ssid[MAX_SSID_LEN+1];
	uint8_t bssid[MAC_ADDR_LEN];
	uint8_t channel;
	uint16_t duration;
}__attribute__((packed));

struct cmd_config_mac_address {
	struct command_header header;
	uint8_t mac_addr[MAC_ADDR_LEN];
}__attribute__((packed));

struct cmd_sta_connect {
	struct command_header header;
	char ssid[MAX_SSID_LEN+1];
	uint8_t bssid[MAC_ADDR_LEN];
	uint8_t channel;
	uint16_t assoc_flags;
	uint8_t is_auth_open;
	uint8_t assoc_ie_len;
	uint8_t assoc_ie[];
}__attribute__((packed));

struct cmd_sta_disconnect {
	struct command_header header;
	uint16_t reason_code;
}__attribute__((packed));

struct wifi_sec_key {
	uint32_t algo;
	uint32_t index;
	uint8_t data[MAX_KEY_LEN];
	uint32_t len;
	uint8_t mac_addr[MAC_ADDR_LEN];
	uint8_t seq[MAX_SEQ_LEN];
	uint32_t seq_len;
	uint8_t del;
	uint8_t set_cur;
	uint8_t pad1[2];
}__attribute__((packed));

struct cmd_key_operation {
	struct command_header header;
	struct wifi_sec_key key;
}__attribute__((packed));

struct event_header {
	uint8_t event_code;
	uint8_t status;
	uint16_t len;
}__attribute__((packed));

struct scan_event {
	struct event_header header;
	uint8_t frame_type;
	uint8_t bssid[MAC_ADDR_LEN];
	uint8_t channel;
	uint32_t rssi;
	uint64_t tsf;
	uint16_t frame_len;
	uint8_t frame[0];
}__attribute__((packed));

struct connect_event {
	struct event_header header;
	char ssid[MAX_SSID_LEN+1];
	uint8_t bssid[MAC_ADDR_LEN];
	uint8_t channel;
}__attribute__((packed));

struct disconnect_event {
	struct event_header header;
	char ssid[MAX_SSID_LEN+1];
	uint8_t bssid[MAC_ADDR_LEN];
	uint8_t reason;
}__attribute__((packed));

struct esp_internal_bootup_event {
	struct event_header header;
	uint8_t	len;
	uint8_t	data[0];
}__attribute__((packed));

struct fw_version {
	uint8_t major1; 
	uint8_t major2; 
	uint8_t minor; 
}__attribute__((packed));

struct fw_data {
	struct fw_version version;
	uint32_t last_reset_reason;
}__attribute__((packed));



static inline uint16_t compute_checksum(uint8_t *buf, uint16_t len)
{
	uint16_t checksum = 0;
	uint16_t i = 0;

	while(i < len) {
		checksum += buf[i];
		i++;
	}

	return checksum;
}

#endif
