#ifndef USB_HEADSET_SETTINGS__H
#define USB_HEADSET_SETTINGS__H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tusb_config.h"

typedef struct _usb_headset_settings_t {
	// Audio controls
	// Current states
	int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0
	int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0
	uint16_t volume_db[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0

	uint32_t sample_rate;
	uint8_t resolution;
	uint32_t blink_interval_ms;

	uint16_t samples_in_i2s_frame_min;
	uint16_t samples_in_i2s_frame_max;

	bool mic_muted_by_user;
	bool mic_live_status;

} usb_headset_settings_t;

#ifdef __cplusplus
}
#endif

#endif //USB_HEADSET_SETTINGS__H
