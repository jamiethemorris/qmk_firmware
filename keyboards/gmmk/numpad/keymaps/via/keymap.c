/* Copyright 2021 Glorious, LLC <salman@pcgamingrace.com>
Modified 2022 by rustedaperture for qmk_firmware
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include QMK_KEYBOARD_H
#include "analog.h"
#include "qmk_midi.h"
#define MACRO(x) (SAFE_RANGE + x)

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

//      NUM      /       *       -
//      7        8       9       +
//      4        5       6       ENCODER
//      1        2       3       RET
//      0                        .

  [0] = LAYOUT(
    MO(1),   KC_PSLS,   KC_PAST,   KC_PMNS,
    KC_P7,   KC_P8,     KC_P9,     KC_PPLS,
    KC_P4,   KC_P5,     KC_P6,     KC_CALC,
    KC_P1,   KC_P2,     KC_P3,     KC_PENT,
    LT(1,KC_P0),                         KC_PDOT
  ),
  [1] = LAYOUT(
    MO(0),   KC_F10,    KC_F11,   KC_F12,
    KC_F7,     KC_F8,   KC_F9,     KC_PPLS,
    MACRO(1),  KC_F5,     MACRO(0),   KC_CALC,
    KC_F1,     KC_F2,   KC_F3,     KC_PENT,
    RGB_TOG,                         QK_BOOT
  ),
  [2] = LAYOUT(
    MI_A1,   MI_As1,   MI_B1,    MI_OCTD,
    MI_Fs1,  MI_G1,    MI_Gs1,   MI_OCTU,
    MI_Ds1,  MI_E1,    MI_F1,    TO(0),
    MI_C1,   MI_Cs1,   MI_D1,    MI_BNDU,  // Enter is now pitch wheel up
    MI_TR0,                         MI_BNDD // Dot is now pitch wheel down
  ),
  [3] = LAYOUT(
    TO(0),   KC_PSLS,   KC_PAST,   KC_PMNS,
    KC_P7,   KC_P8,     KC_P9,     KC_PPLS,
    KC_P4,   KC_P5,     KC_P6,     KC_CALC,
    KC_P1,   KC_P2,     KC_P3,     KC_PENT,
    KC_P0,                         KC_PDOT
  ),
};
// clang-format on

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MACRO(0):
            if (record->event.pressed) {
                // when MACRO(0) is pressed
                tap_code16(LGUI(KC_LSFT));
                tap_code(KC_F5);
            }
            return false;  // Skip all further processing of this key
        case MACRO(1):
            if (record->event.pressed) {
                // when MACRO(1) is pressed
                tap_code16(KC_LSFT);
                tap_code(KC_F5);
            }
            return false;  // Skip all further processing of this key
        default:
            return true;  // Process all other keycodes normally
    }
}

const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [0] = { ENCODER_CCW_CW(KC_COMM, KC_DOT) },
    [1] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [2] = { ENCODER_CCW_CW(MI_TRSD, MI_TRSU) },
    [3] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) }
};

// Potentiometer Slider, MIDI Control

uint8_t divisor = 0;

uint8_t last_read = 0;
 
void slider(void) {
    uint8_t current_read = (analogReadPin(SLIDER_PIN) +last_read)/8; //filter strength
 
    if (current_read != last_read ) {
        midi_send_cc(&midi_device, 1, 0x01, 0x7F - (analogReadPin(SLIDER_PIN) >>3));
 
    last_read = current_read;
    }
 
}

 
void matrix_scan_user(void) {
    slider();
}