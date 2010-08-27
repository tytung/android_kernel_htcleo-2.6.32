/* arch/arm/mach-msm/pmic_global.h
 *
 * Author: Markinus
 * The entry header file for all pm functions. If we have differences between devices then
 * we have to replace a define with a function and put it in the c file.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "pmic.h"
#include "dex_comm.h"

// direct defines to functions, if same for all amss
#define pmic_glb_lp_mode_control(a,b) pmic_lp_mode_control(a, b)
#define pmic_glb_secure_mpp_control_digital_output(a, b, c) pmic_secure_mpp_control_digital_output(a, b, c)
#define pmic_glb_secure_mpp_config_i_sink(a, b, c) pmic_secure_mpp_config_i_sink(a, b, c)
#define pmic_glb_secure_mpp_config_digital_input(a, b, c) pmic_secure_mpp_config_digital_input(a, b, c)

#define pmic_glb_speaker_cmd(a) pmic_speaker_cmd(a)
#define pmic_glb_set_spkr_configuration(a) pmic_set_spkr_configuration(a)
#define pmic_glb_spkr_en_right_chan(a) pmic_spkr_en_right_chan(a)
#define pmic_glb_spkr_en_left_chan(a) pmic_spkr_en_left_chan(a)
#define pmic_glb_spkr_en(a, b) pmic_spkr_en(a, b)
#define pmic_glb_spkr_set_gain(a, b) pmic_spkr_set_gain(a, b)
#define pmic_glb_set_speaker_gain(a) pmic_set_speaker_gain(a)
#define pmic_glb_set_speaker_delay(a) pmic_set_speaker_delay(a)
#define pmic_glb_speaker_1k6_zin_enable(a) pmic_speaker_1k6_zin_enable(a)
#define pmic_glb_spkr_set_mux_hpf_corner_freq(a) pmic_spkr_set_mux_hpf_corner_freq(a)
#define pmic_glb_spkr_select_usb_with_hpf_20hz(a) pmic_spkr_select_usb_with_hpf_20hz(a)
#define pmic_glb_spkr_bypass_mux(a) pmic_spkr_bypass_mux(a)
#define pmic_glb_spkr_en_hpf(a) pmic_spkr_en_hpf(a)
#define pmic_glb_spkr_en_sink_curr_from_ref_volt_cir(a) pmic_spkr_en_sink_curr_from_ref_volt_cir(a)
#define pmic_glb_spkr_set_delay(a, b) pmic_spkr_set_delay(a, b)
#define pmic_glb_spkr_en_mute(a, b) pmic_spkr_en_mut(a, b)
#define pmic_glb_mic_en(a) pmic_mic_en(a)
#define pmic_glb_mic_set_volt(a) pmic_mic_set_volt(a)
#define pmic_glb_set_led_intensity(a, b) pmic_set_led_intensity(a, b)
#define pmic_glb_flash_led_set_current(a) pmic_flash_led_set_current(a)
#define pmic_glb_flash_led_set_mode(a) pmic_flash_led_set_mode(a)
#define pmic_glb_flash_led_set_polarity(a) pmic_flash_led_set_polarity(a)
#define pmic_glb_spkr_add_right_left_chan pmic_spkr_add_right_left_chan(uint enable)
#define pmic_glb_spkr_en_stereo(a) pmic_spkr_en_stereo(a)
#define pmic_glb_vib_mot_set_volt(a) pmic_vib_mot_set_volt(a)
#define pmic_glb_vib_mot_set_mode(a) pmic_vib_mot_set_mode(a)
#define pmic_glb_vib_mot_set_polarity(a) pmic_vib_mot_set_polarity(a)
#define pmic_glb_vid_e(a)n pmic_vid_en(a)
#define pmic_glb_vid_load_detect_en(a) pmic_vid_load_detect_en(a)

// Funtion declaration switching dependend on amss version
int pmic_glb_set_vreg(int enable, enum vreg_id id);
int pmic_glb_vreg_set_level(enum vreg_id id, unsigned millivolt); 
int pmic_glb_power_down(void);
int pmic_glb_reset_chip(unsigned restart_reason);