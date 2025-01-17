 /* Copyright 2020 Josef Adamcik
  * Modification for VIA support and RGB underglow by Jens Bonk-Wiltfang
  * 
  * This program is free software: you can redistribute it and/or modify 
  * it under the terms of the GNU General Public License as published by 
  * the Free Software Foundation, either version 2 of the License, or 
  * (at your option) any later version. 
  * 
  * This program is distributed in the hope that it will be useful, 
  * but WITHOUT ANY WARRANTY; without even the implied warranty of 
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
  * GNU General Public License for more details. 
  * 
  * You should have received a copy of the GNU General Public License 
  * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
  */ 

#include QMK_KEYBOARD_H
#include "oled.c"
#ifdef RP2040_BUILD
    #include "qmk_midi.h"
#endif

bool is_ag_swapped = false;
bool is_gui_disabled = false;
static bool alt_gui_swapped = false;
static bool gui_disabled = false;
static bool md_win_active = false;
static bool md_mac_active = false;
static bool md_game_active = false;

enum sofle_layers {
    _DEFAULTS = 0,
    _QWERTY = 0,
    _LOWER,
    _RAISE,
    _MOUSE,
    _NUMPAD,
#ifdef RP2040_BUILD
    _MIDI,
    _CHORD1,
    _CHORD2,
#else
    _MIDI
#endif
};

enum custom_keycodes {
    MD_WIN = QK_KB_0,
    MD_MAC,
    MD_GAME,
    #ifdef RP2040_BUILD
    C_MAJ,
    D_MIN,
    E_MIN,
    F_MAJ,
    G_MAJ,
    A_MIN,
    B_DIM,
    
    MI_CHOCTU,
    MI_CHOCTD,
    MI_CHTRSU,
    MI_CHTRSD,
    
    MI_MDLT,
    MI_7TH,
    MODWL_UP,
    MODWL_DN,
    #endif
	
	DEBUG,
	RESTART_DEBUG,
	STOP_DEBUG

};

#ifdef RP2040_BUILD
static int8_t octave = 0;  // starts at 0, can be increased or decreased
static int8_t transpose = 0;  // starts at 0, can be increased or decreased

static bool is_key_8_held = false;
static bool is_key_9_held = false;

uint8_t mod_wheel_val = 0;

typedef struct {
    int8_t octave;
    int8_t transpose;
} ChordData;

ChordData chordData[7];
#endif

#define LT_TAB_NP LT(_NUMPAD,KC_TAB)
#define LT_SPC_RS LT(_RAISE,KC_SPC)
#define LT_ENT_LW LT(_LOWER,KC_ENT)
#define LT_ENT_MS LT(_MOUSE,KC_ENT)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/*
 * QWERTY
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * | GESC |   1  |   2  |   3  |   4  |   5  |                    |   6  |   7  |   8  |   9  |   0  | Bspc |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |Tab/NP|   Q  |   W  |   E  |   R  |   T  |                    |   Y  |   U  |   I  |   O  |   P  |  \   |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * | Caps |   A  |   S  |   D  |   F  |   G  |-------.    ,-------|   H  |   J  |   K  |   L  |   ;  |  '   |
 * |------+------+------+------+------+------|       |    |       |------+------+------+------+------+------|
 * | Shift|   Z  |   X  |   C  |   V  |   B  |-------|    |-------|   N  |   M  |   ,  |   .  |   /  | Enter|
 * `-----------------------------------------/       /     \      \-----------------------------------------'
 *            | LCTR | LAlt | LGUI |LOWER | /Enter  /       \Spc/Rs\  |MOUSE |  -   |  [   |   ]  |
 *            |      |      |      |      |/       /         \      \ |      |      |      |      |
 *            `----------------------------------'           '------''---------------------------'
 */

/*

TAP DANCE: 
0: W/Win
1: M/Mac
2: G/Game 
3: Minus/Equal
4: Del/MIDI Layer
5: Chord1
6: Chord2

*/

[_QWERTY] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  QK_GESC,   KC_1,   KC_2,    KC_3,    KC_4,    KC_5,                       KC_6,    KC_7,    KC_8,    KC_9,    KC_0,  KC_BSPC,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  LT_TAB_NP,KC_Q,  KC_W,   KC_E,    KC_R,    KC_T,                       KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    TD(3),
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  KC_HYPR, KC_A,   KC_S,    KC_D,    KC_F,    KC_G,                      KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------| 
  KC_LSFT, KC_Z,   KC_X,    KC_C,    KC_V,    KC_B,   KC_MUTE,    XXXXXXX,KC_N,    KC_M,  KC_COMM,  KC_DOT, KC_SLSH,  TD(4), //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
                 KC_LCTL,KC_LALT,KC_LGUI, KC_SPC, LT_ENT_LW,     LT_ENT_MS,LT_SPC_RS,KC_BSLS,KC_LBRC,KC_RBRC                
  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),

[_LOWER] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  KC_GRV,  KC_F1,   KC_F2,  KC_F3,   KC_F4,   KC_F5,                      KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, KC_1,    KC_2,   KC_3,    KC_4,    KC_5,                       KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_F12,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  KC_CAPS, KC_EXLM,KC_AT,   KC_HASH, KC_DLR,  KC_PERC,                    KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_PIPE,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
  _______, KC_EQL, KC_MINS, KC_PLUS, KC_LCBR, KC_RCBR,_______,    _______,KC_LBRC, KC_RBRC, KC_SCLN, KC_COLN, KC_BSLS, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
                 _______, _______, _______, _______,  _______,    _______, _______, _______, _______, _______

  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),

[_RAISE] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, KC_DEL,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, _______,                   _______, _______, KC_UP,   _______, _______, _______,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, KC_PGUP,                   _______, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, KC_PGDN,_______,   _______,_______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
                 _______, _______, _______, _______,  _______,    _______, _______, _______, _______, _______
  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),

[_MOUSE] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______,KC_DEL,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, _______,                   _______, KC_BTN4, KC_MS_U, KC_BTN5, _______, _______,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, KC_WH_U,                   KC_BTN1, KC_MS_L, KC_MS_D, KC_MS_R, KC_BTN2, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, KC_WH_D,_______,   _______,_______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
                 _______, _______, _______, _______,  _______,    _______, _______, _______, _______, _______
  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),

[_NUMPAD] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,                   KC_HOME, KC_NUM, KC_PSLS, KC_PAST, KC_PMNS, _______,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,                   KC_END,  KC_P7,  KC_P8,   KC_P9,   KC_PPLS, XXXXXXX,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,                   KC_PGUP, KC_P4,  KC_P5,   KC_P6,   KC_PPLS, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,_______,   _______,KC_PGDN, KC_P1,  KC_P2,   KC_P3,   KC_PENT, _______,  
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
                 _______, _______, _______, _______,  _______,    _______, _______,  KC_P0,   KC_PDOT, KC_PENT
  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),

#ifdef RP2040_BUILD
[_MIDI] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  MI_TRSU,  MI_Ds1, MI_E1,   MI_F1,   MI_Fs1,  MI_G1,                     MI_Ds2, MI_E2,   MI_F2,   MI_Fs2,  MI_G2, MI_TRSU,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  MI_TRSD,  MI_As,  MI_B,    MI_C1,  MI_Cs1,   MI_D1,                     MI_As1,  MI_B1,  MI_C2,   MI_Cs2,  MI_D2, MI_TRSD,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  TD(5),    MI_F,   MI_Fs,   MI_G,   MI_Gs,    MI_A,                      MI_F1,   MI_Fs1, MI_G1,   MI_Gs1,  MI_A1, TD(6),
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
  TO(0),  MI_C,   MI_Cs,   MI_D,   MI_Ds,    MI_E,  _______,    _______,MI_C1,   MI_Cs1, MI_D1,   MI_Ds1,  MI_E1, _______,  
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
                  MI_OCTD, MI_OCTU, MI_BNDD, MI_BNDU, _______,    _______, _______,  KC_P0,   KC_PDOT, TO(0)
  //            \--------+--------+--------+---------+-------|   |--------+---------+--------+---------+-------/
),
[_CHORD1] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  MI_CHTRSU,_______,_______, _______, _______, _______,                   D_MIN,   E_MIN,  F_MAJ,   MI_MDLT, MI_7TH, MI_CHTRSU,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  MI_CHTRSD,_______,_______, _______, _______, _______,                   F_MAJ,  G_MAJ,   A_MIN,   B_DIM,   C_MAJ, MI_CHTRSD,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  TG(5),    _______,_______, _______, _______, _______,                   A_MIN,  B_DIM,   C_MAJ,   D_MIN,   E_MIN, TG(5),
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
  TO(0),  _______,_______, _______, _______, _______,_______,   _______,C_MAJ,  D_MIN,   E_MIN,   F_MAJ,   G_MAJ, _______,  
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
                  MI_CHOCTD, MI_CHOCTU, MI_BNDD, MI_BNDU, _______,    _______, _______,  KC_P0,   KC_PDOT, TO(0)
  //            \--------+--------+--------+---------+-------|   |--------+---------+--------+---------+-------/
),
[_CHORD2] = LAYOUT(
  //,------------------------------------------------.                    ,---------------------------------------------------.
  MI_CHTRSU,D_MIN,  E_MIN,   F_MAJ,   MI_MDLT, MI_7TH,                     _______,_______, _______, _______, _______, MI_CHTRSU,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  MI_CHTRSD,F_MAJ,  G_MAJ,   A_MIN,   B_DIM,   C_MAJ,                     _______,_______, _______, _______, _______, MI_CHTRSD,
  //|------+-------+--------+--------+--------+------|                   |--------+-------+--------+--------+--------+---------|
  TG(5),    A_MIN,  B_DIM,   C_MAJ,   D_MIN,   E_MIN,                     _______,_______, _______, _______, _______, TG(5),
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
  TO(0),  C_MAJ,  D_MIN,   E_MIN,   F_MAJ,   G_MAJ, _______,    _______,_______,_______, _______, _______, _______, _______,  
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+-------+--------+--------+--------+---------|
                  MI_CHOCTD, MI_CHOCTU, MI_BNDD, MI_BNDU, _______,    _______, _______,  KC_P0,   KC_PDOT, TO(0)
  //            \--------+--------+--------+---------+-------|   |--------+---------+--------+---------+-------/
),
#else 
[_MIDI] = LAYOUT(
//,------------------------------------------------.                    ,---------------------------------------------------.
  _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|                   |--------+--------+--------+--------+--------+---------|
  _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
  TO(0), _______, _______, _______, _______, _______,_______,   _______,_______, _______, _______, _______, _______, _______,
  //|------+-------+--------+--------+--------+------|  ===  |   |  ===  |--------+--------+--------+--------+--------+---------|
                 _______, _______, _______, _______,  _______,    _______, _______, _______, _______, _______
  //           \--------+--------+--------+---------+--------|   |--------+--------+--------+---------+-------/
),
#endif
};

#ifdef ENCODER_MAP_ENABLE
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [0] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU),           ENCODER_CCW_CW(KC_COMM, KC_DOT) }, 
    [1] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(_______, _______) },
    [2] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(_______, _______) },
    [3] = { ENCODER_CCW_CW(KC_WH_D, KC_WH_U),           ENCODER_CCW_CW(KC_WH_D, KC_WH_U) },
    [4] = { ENCODER_CCW_CW(RGB_HUD, RGB_HUI),           ENCODER_CCW_CW(RGB_SAD, RGB_SAI) },
    #ifdef RP2040_BUILD
        [5] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(MODWL_DN, MODWL_UP) },
        [6] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(MODWL_DN, MODWL_UP) },
        [7] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(MODWL_DN, MODWL_UP) },
    #else
        [5] = { ENCODER_CCW_CW(_______, _______),           ENCODER_CCW_CW(_______, _______) },
    #endif
    // [3] = { ENCODER_CCW_CW(RGB_HUD, RGB_HUI),           ENCODER_CCW_CW(RGB_SAD, RGB_SAI) },
};
#endif

void keyboard_post_init_user(void) {

    // debug_enable=true;
    // debug_matrix=true;
    // debug_keyboard=true;
    //debug_mouse=true;
}

enum {
  W_WIN = 0,
  M_MAC,
  G_GAME,
  MIN_EQL,
  DEL_MIDI,
#ifdef RP2040_BUILD
  LYR_CHRDR,
  LYR_CHRDL
#endif
};

typedef struct {
  bool is_press_action;
  int state;
} tap;

enum {
  SINGLE_TAP = 1,
  SINGLE_HOLD = 2,
  DOUBLE_TAP = 3,
  DOUBLE_HOLD = 4,
  DOUBLE_SINGLE_TAP = 5,
  TRIPLE_TAP = 6,
  TRIPLE_HOLD = 7
};

int cur_dance (tap_dance_state_t *state) {
  if (state->count == 1) {
    if (state->interrupted || !state->pressed)  return SINGLE_TAP;
    //key has not been interrupted, but they key is still held. Means you want to send a 'HOLD'.
    else return SINGLE_HOLD;
  }
  else if (state->count == 2) {
    /*
     * DOUBLE_SINGLE_TAP is to distinguish between typing "pepper", and actually wanting a double tap
     * action when hitting 'pp'. Suggested use case for this return value is when you want to send two
     * keystrokes of the key, and not the 'double tap' action/macro.
    */
    if (state->interrupted) return DOUBLE_SINGLE_TAP;
    else if (state->pressed) return DOUBLE_HOLD;
    else return DOUBLE_TAP;
  }
  //Assumes no one is trying to type the same letter three times (at least not quickly).
  //If your tap dance key is 'KC_W', and you want to type "www." quickly - then you will need to add
  //an exception here to return a 'TRIPLE_SINGLE_TAP', and define that enum just like 'DOUBLE_SINGLE_TAP'
  if (state->count == 3) {
    if (state->interrupted || !state->pressed)  return TRIPLE_TAP;
    else return TRIPLE_HOLD;
  }
  else return 8; //magic number. At some point this method will expand to work for more presses
}

//instanalize an instance of 'tap' for the tap dance.
static tap wtap_state = {.is_press_action = true, .state = 0};
static tap mtap_state = {.is_press_action = true, .state = 0};
static tap gtap_state = {.is_press_action = true, .state = 0};
static tap mins_tap_state = {.is_press_action = true, .state = 0};
static tap del_tap_state = {.is_press_action = true, .state = 0};
#ifdef RP2040_BUILD
static tap chrdr_tap_state = {.is_press_action = true, .state = 0};
static tap chrdl_tap_state = {.is_press_action = true, .state = 0};
#endif

void w_finished (tap_dance_state_t *state, void *user_data) {
  wtap_state.state = cur_dance(state);
  switch (wtap_state.state) {
    case SINGLE_TAP: register_code(KC_W); break;
    // case SINGLE_HOLD: register_code(KC_NO); break;
    // case DOUBLE_TAP: register_code(KC_NO); break;
    case DOUBLE_HOLD: md_win_active = true; break;
    case DOUBLE_SINGLE_TAP: register_code(KC_W); unregister_code(KC_W); register_code(KC_W);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}

void w_reset (tap_dance_state_t *state, void *user_data) {
  switch (wtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_W); break;
    // case SINGLE_HOLD: unregister_code(KC_NO); break;
    // case DOUBLE_TAP: unregister_code(KC_NO); break;
    case DOUBLE_HOLD: md_win_active = false; break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_W);
  }
  wtap_state.state = 0;
}

void m_finished (tap_dance_state_t *state, void *user_data) {
  mtap_state.state = cur_dance(state);
  switch (mtap_state.state) {
    case SINGLE_TAP: register_code(KC_M); break;
    // case SINGLE_HOLD: register_code(KC_NO); break;
    // case DOUBLE_TAP: register_code(KC_NO); break;
    case DOUBLE_HOLD: md_mac_active = true; break;
    case DOUBLE_SINGLE_TAP: register_code(KC_M); unregister_code(KC_M); register_code(KC_M);
  }
}

void m_reset (tap_dance_state_t *state, void *user_data) {
  switch (mtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_M); break;
    // case SINGLE_HOLD: unregister_code(KC_NO); break;
    // case DOUBLE_TAP: unregister_code(KC_NO); break;
    case DOUBLE_HOLD: md_mac_active = false; break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_M);
  }
  mtap_state.state = 0;
}

void g_finished (tap_dance_state_t *state, void *user_data) {
  gtap_state.state = cur_dance(state);
  switch (cur_dance(state)) {
    case SINGLE_TAP: register_code(KC_G); break;
    // case SINGLE_HOLD: register_code(KC_NO); break;
    // case DOUBLE_TAP: register_code(KC_NO); break;
    case DOUBLE_HOLD: md_game_active = true; break;
    case DOUBLE_SINGLE_TAP: register_code(KC_G); unregister_code(KC_G); register_code(KC_G);
  }
}

void g_reset (tap_dance_state_t *state, void *user_data) {
  switch (gtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_G); break;
    // case SINGLE_HOLD: unregister_code(KC_NO); break;
    // case DOUBLE_TAP: unregister_code(KC_NO); break;
    case DOUBLE_HOLD: md_game_active = false; break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_G);
  }
  gtap_state.state = 0;
}

void mins_finished (tap_dance_state_t *state, void *user_data) {
  mins_tap_state.state = cur_dance(state);
  switch (mins_tap_state.state) {
    case SINGLE_TAP: register_code(KC_MINS); break;
    case SINGLE_HOLD: register_code(KC_EQL); break;
    // case DOUBLE_TAP: register_code(KC_NO); break;
    // case DOUBLE_HOLD: register_code(KC_EQL); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_MINS); unregister_code(KC_MINS); register_code(KC_MINS);
  }
}

void mins_reset (tap_dance_state_t *state, void *user_data) {
  switch (mins_tap_state.state) {
    case SINGLE_TAP: unregister_code(KC_MINS); break;
    case SINGLE_HOLD: unregister_code(KC_EQL); break;
    // case DOUBLE_TAP: unregister_code(KC_NO); break;
    // case DOUBLE_HOLD: unregister_code(KC_EQL); break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_MINS);
  }
  mins_tap_state.state = 0;
}

void del_finished (tap_dance_state_t *state, void *user_data) {
  del_tap_state.state = cur_dance(state);
  switch (del_tap_state.state) {
    case SINGLE_TAP: 
      tap_code(KC_DEL); 
      break;
    /* case SINGLE_HOLD: 
      layer_on(_MIDI); 
      break;
    case DOUBLE_TAP: 
      //check to see if the layer is already set
      if (layer_state_is(_MIDI)) {
        //if already set, then switch it off
        layer_off(_MIDI);
      } else { 
        //if not already set, then switch the layer on
        layer_on(_MIDI);
      }
      break; */
    case DOUBLE_HOLD: 
      layer_on(_MIDI); 
      break;
  }
}

void del_reset (tap_dance_state_t *state, void *user_data) {
  //if the key was held down and now is released then switch off the layer
  if (del_tap_state.state==SINGLE_HOLD) {
    layer_off(_MIDI);
  }
  del_tap_state.state = 0;
}

#ifdef RP2040_BUILD
void chrdr_finished (tap_dance_state_t *state, void *user_data) {
  chrdr_tap_state.state = cur_dance(state);
  switch (chrdr_tap_state.state) {
    case SINGLE_TAP: 
      //check to see if the layer is already set
      if (layer_state_is(_CHORD1)) {
        //if already set, then switch it off
        layer_off(_CHORD1);
      } else { 
        //if not already set, then switch the layer on
        layer_on(_CHORD1);
      }
      break; 
    /* case SINGLE_HOLD: 
      layer_on(_MIDI); 
      break; */
    case SINGLE_HOLD: 
      layer_on(_CHORD1); 
      break;
  }
}

void chrdr_reset (tap_dance_state_t *state, void *user_data) {
  //if the key was held down and now is released then switch off the layer
  if (chrdr_tap_state.state==SINGLE_HOLD) {
    layer_off(_CHORD1);
  }
  chrdr_tap_state.state = 0;
}

void chrdl_finished (tap_dance_state_t *state, void *user_data) {
  chrdl_tap_state.state = cur_dance(state);
  switch (chrdl_tap_state.state) {
    case SINGLE_TAP: 
      //check to see if the layer is already set
      if (layer_state_is(_CHORD2)) {
        //if already set, then switch it off
        layer_off(_CHORD2);
      } else { 
        //if not already set, then switch the layer on
        layer_on(_CHORD2);
      }
      break; 
    /* case SINGLE_HOLD: 
      layer_on(_MIDI); 
      break; */
    case SINGLE_HOLD: 
      layer_on(_CHORD2); 
      break;
  }
}

void chrdl_reset (tap_dance_state_t *state, void *user_data) {
  //if the key was held down and now is released then switch off the layer
  if (chrdl_tap_state.state==SINGLE_HOLD) {
    layer_off(_CHORD2);
  }
  chrdl_tap_state.state = 0;
}

#endif

tap_dance_action_t tap_dance_actions[] = {
    [W_WIN]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, w_finished, w_reset),
    [M_MAC]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, m_finished, m_reset),
    [G_GAME]  = ACTION_TAP_DANCE_FN_ADVANCED(NULL, g_finished, g_reset),
    [MIN_EQL] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, mins_finished, mins_reset),
    [DEL_MIDI]= ACTION_TAP_DANCE_FN_ADVANCED(NULL, del_finished, del_reset),
#ifdef RP2040_BUILD
    [LYR_CHRDR] = ACTION_TAP_DANCE_FN_ADVANCED(NULL,chrdr_finished, chrdr_reset),
    [LYR_CHRDL] = ACTION_TAP_DANCE_FN_ADVANCED(NULL,chrdl_finished, chrdl_reset)
#endif
};

void swap_keys(void) {
    if (!alt_gui_swapped) {
        // Code to swap ALT and GUI here
        alt_gui_swapped = true;
    }
}

void unswap_keys(void) {
    if (alt_gui_swapped) {
        // Code to unswap ALT and GUI here
        alt_gui_swapped = false;
    }
}

void toggle_gui(void) {
    gui_disabled = !gui_disabled;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    if (alt_gui_swapped) {
        if (record->event.pressed) {
            switch (keycode) {
                case KC_LALT:
                    register_code(KC_LGUI);
                    return false;
                case KC_LGUI:
                    register_code(KC_LALT);
                    return false;
                case KC_RALT:
                    register_code(KC_RGUI);
                    return false;
                case KC_RGUI:
                    register_code(KC_RALT);
                    return false;
            }
        } else {
            switch (keycode) {
                case KC_LALT:
                    unregister_code(KC_LGUI);
                    return false;
                case KC_LGUI:
                    unregister_code(KC_LALT);
                    return false;
                case KC_RALT:
                    unregister_code(KC_RGUI);
                    return false;
                case KC_RGUI:
                    unregister_code(KC_RALT);
                    return false;
            }
        }
    }
    if (gui_disabled) {
        if (keycode == KC_LGUI || keycode == KC_RGUI) {
            return false;  // Block the GUI key
        }
    } else {
        if (keycode == KC_LGUI || keycode == KC_RGUI) {
            return true;  // Unblock the GUI key
        }
    }
    
    switch (keycode) {	
	// Normal macros
        case MD_WIN:
            if (md_game_active){
                if (record->event.pressed) {
                    swap_keys();
                    gui_disabled = false;  // Re-enable the GUI key
                    #ifdef OLED_ENABLE
                    is_ag_swapped = alt_gui_swapped;
                    is_gui_disabled = gui_disabled;
                    #endif
                }
            }
            return false;
        case MD_MAC:
            if (md_mac_active){
                if (record->event.pressed) {
                    unswap_keys();
                    gui_disabled = false;  // Re-enable the GUI key
                    #ifdef OLED_ENABLE
                    is_ag_swapped = alt_gui_swapped;
                    is_gui_disabled = gui_disabled;
                    #endif
                }
            }
            return false;
        case MD_GAME:
            if (md_game_active){
                if (record->event.pressed) {
                    toggle_gui();
                    #ifdef OLED_ENABLE
                    is_gui_disabled = gui_disabled;
                    #endif
                }
            }
            return false;
        case DEBUG:
            if (record->event.pressed) {
                tap_code(KC_F5);
            }
            return false;  
        case RESTART_DEBUG:
            if (record->event.pressed) {
                register_code(KC_LGUI);
				register_code(KC_LSFT);
				tap_code(KC_F5);
				unregister_code(KC_LSFT);
				unregister_code(KC_LGUI);
            }
            return false;  
        case STOP_DEBUG:
            if (record->event.pressed) {
                register_code(KC_LSFT);
				tap_code(KC_F5);
				unregister_code(KC_LSFT);
            }
            return false;  
#ifdef RP2040_BUILD

	// Transposition
		case MI_CHOCTU:
			if (record->event.pressed) {
				octave += 12;  // increase by 12 to represent an octave
			}
			return false;
		case MI_CHOCTD:
			if (record->event.pressed) {
				octave -= 12;  // decrease by 12 to represent an octave
			}
			return false;
		case MI_CHTRSU:
			if (record->event.pressed) {
				transpose += 1;  // transpose up by 1 semitone
			}
			return false;
		case MI_CHTRSD:
			if (record->event.pressed) {
				transpose -= 1;  // transpose down by 1 semitone
			}
			return false;
			
		case MI_7TH:  // add appropriate 7th note to a chord
			if (record->event.pressed) {
				is_key_9_held = true;
			} else {
				is_key_9_held = false;
			}
			return false;

		case MI_MDLT:  // sharpen 3rd half step for minor, flatten 3rd half step for major, and flatten diminished 3rd by half step
			if (record->event.pressed) {
				is_key_8_held = true;
			} else {
				is_key_8_held = false;
			}
			return false;
		
	// C Major scale chords
		case C_MAJ:
			if (record->event.pressed) {
				chordData[0].octave = octave;
				chordData[0].transpose = transpose;

				// Base notes (root and 5th)
				midi_send_noteon(&midi_device, 0, 60 + chordData[0].octave + chordData[0].transpose, 127);  // C note
				midi_send_noteon(&midi_device, 0, 67 + chordData[0].octave + chordData[0].transpose, 127);  // G note

				if (is_key_8_held) {
					// Send the minor 3rd for C minor
					midi_send_noteon(&midi_device, 0, 63 + chordData[0].octave + chordData[0].transpose, 127);  // E flat
				} else {
					// Send the major 3rd for C major
					midi_send_noteon(&midi_device, 0, 64 + chordData[0].octave + chordData[0].transpose, 127);  // E
				}

				if (is_key_9_held) {
					// Send the 7th for C major 7
					midi_send_noteon(&midi_device, 0, 71 + chordData[0].octave + chordData[0].transpose, 127);  // B
				}
			} else {
				// Send note off for all potential notes (this ensures no hanging notes)
				midi_send_noteoff(&midi_device, 0, 60 + chordData[0].octave + chordData[0].transpose, 0);  // C note
				midi_send_noteoff(&midi_device, 0, 63 + chordData[0].octave + chordData[0].transpose, 0);  // E flat
				midi_send_noteoff(&midi_device, 0, 64 + chordData[0].octave + chordData[0].transpose, 0);  // E
				midi_send_noteoff(&midi_device, 0, 67 + chordData[0].octave + chordData[0].transpose, 0);  // G note
				midi_send_noteoff(&midi_device, 0, 71 + chordData[0].octave + chordData[0].transpose, 0);  // B
			}
			return false;
        case D_MIN:
			if (record->event.pressed) {
				chordData[1].octave = octave;
				chordData[1].transpose = transpose;

				// Base notes (root and 5th)
				midi_send_noteon(&midi_device, 0, 62 + chordData[1].octave + chordData[1].transpose, 127);  // D note
				midi_send_noteon(&midi_device, 0, 69 + chordData[1].octave + chordData[1].transpose, 127);  // A note

				if (is_key_8_held) {
					// Send the major 3rd for D major
					midi_send_noteon(&midi_device, 0, 66 + chordData[1].octave + chordData[1].transpose, 127);  // F sharp
				} else {
					// Send the minor 3rd for D minor
					midi_send_noteon(&midi_device, 0, 65 + chordData[1].octave + chordData[1].transpose, 127);  // F
				}

				if (is_key_9_held) {
					// Send the 7th for D minor 7
					midi_send_noteon(&midi_device, 0, 72 + chordData[1].octave + chordData[1].transpose, 127);  // C
				}
			} else {
				// Send note off for all potential notes (this ensures no hanging notes)
				midi_send_noteoff(&midi_device, 0, 62 + chordData[1].octave + chordData[1].transpose, 0);  // D note
				midi_send_noteoff(&midi_device, 0, 65 + chordData[1].octave + chordData[1].transpose, 0);  // F
				midi_send_noteoff(&midi_device, 0, 66 + chordData[1].octave + chordData[1].transpose, 0);  // F sharp
				midi_send_noteoff(&midi_device, 0, 69 + chordData[1].octave + chordData[1].transpose, 0);  // A note
				midi_send_noteoff(&midi_device, 0, 72 + chordData[1].octave + chordData[1].transpose, 0);  // C
			}
			return false;

		case E_MIN:          
			if (record->event.pressed) {
				chordData[2].octave = octave;
				chordData[2].transpose = transpose;

				// Base notes (root and 5th)
				midi_send_noteon(&midi_device, 0, 64 + chordData[2].octave + chordData[2].transpose, 127);  // E note
				midi_send_noteon(&midi_device, 0, 71 + chordData[2].octave + chordData[2].transpose, 127);  // B note

				if (is_key_8_held) {
					// Send the major 3rd for E major
					midi_send_noteon(&midi_device, 0, 68 + chordData[2].octave + chordData[2].transpose, 127);  // G sharp
				} else {
					// Send the minor 3rd for E minor
					midi_send_noteon(&midi_device, 0, 67 + chordData[2].octave + chordData[2].transpose, 127);  // G
				}

				if (is_key_9_held) {
					// Send the 7th for E minor 7
					midi_send_noteon(&midi_device, 0, 74 + chordData[2].octave + chordData[2].transpose, 127);  // D
				}
			} else {
				// Send note off for all potential notes (this ensures no hanging notes)
				midi_send_noteoff(&midi_device, 0, 64 + chordData[2].octave + chordData[2].transpose, 0);  // E note
				midi_send_noteoff(&midi_device, 0, 67 + chordData[2].octave + chordData[2].transpose, 0);  // G
				midi_send_noteoff(&midi_device, 0, 68 + chordData[2].octave + chordData[2].transpose, 0);  // G sharp
				midi_send_noteoff(&midi_device, 0, 71 + chordData[2].octave + chordData[2].transpose, 0);  // B note
				midi_send_noteoff(&midi_device, 0, 74 + chordData[2].octave + chordData[2].transpose, 0);  // D
			}
			return false;
		case F_MAJ:
			if (record->event.pressed) {
				chordData[3].octave = octave;
				chordData[3].transpose = transpose;

				// Base notes (root and 5th)
				midi_send_noteon(&midi_device, 0, 65 + chordData[3].octave + chordData[3].transpose, 127);  // F note
				midi_send_noteon(&midi_device, 0, 72 + chordData[3].octave + chordData[3].transpose, 127);  // C note

				if (is_key_8_held) {
					// Send the minor 3rd for F minor
					midi_send_noteon(&midi_device, 0, 68 + chordData[3].octave + chordData[3].transpose, 127);  // A flat
				} else {
					// Send the major 3rd for F major
					midi_send_noteon(&midi_device, 0, 69 + chordData[3].octave + chordData[3].transpose, 127);  // A
				}

				if (is_key_9_held) {
					// Send the major 7th for F major 7
					midi_send_noteon(&midi_device, 0, 76 + chordData[3].octave + chordData[3].transpose, 127);  // E
				}
			} else {
				// Send note off for all potential notes (this ensures no hanging notes)
				midi_send_noteoff(&midi_device, 0, 65 + chordData[3].octave + chordData[3].transpose, 0);  // F note
				midi_send_noteoff(&midi_device, 0, 68 + chordData[3].octave + chordData[3].transpose, 0);  // A flat
				midi_send_noteoff(&midi_device, 0, 69 + chordData[3].octave + chordData[3].transpose, 0);  // A
				midi_send_noteoff(&midi_device, 0, 72 + chordData[3].octave + chordData[3].transpose, 0);  // C note
				midi_send_noteoff(&midi_device, 0, 76 + chordData[3].octave + chordData[3].transpose, 0);  // E
			}
			return false;

		case G_MAJ:
			if (record->event.pressed) {
				chordData[4].octave = octave;
				chordData[4].transpose = transpose;

				// Base notes (root and 5th)
				midi_send_noteon(&midi_device, 0, 67 + chordData[4].octave + chordData[4].transpose, 127);  // G note
				midi_send_noteon(&midi_device, 0, 74 + chordData[4].octave + chordData[4].transpose, 127);  // D note

				if (is_key_8_held) {
					// Send the minor 3rd for G minor
					midi_send_noteon(&midi_device, 0, 70 + chordData[4].octave + chordData[4].transpose, 127);  // B flat
				} else {
					// Send the major 3rd for G major
					midi_send_noteon(&midi_device, 0, 71 + chordData[4].octave + chordData[4].transpose, 127);  // B
				}

				if (is_key_9_held) {
					// Send the minor 7th for G dominant 7
					midi_send_noteon(&midi_device, 0, 77 + chordData[4].octave + chordData[4].transpose, 127);  // F
				}
			} else {
				// Send note off for all potential notes (this ensures no hanging notes)
				midi_send_noteoff(&midi_device, 0, 67 + chordData[4].octave + chordData[4].transpose, 0);  // G note
				midi_send_noteoff(&midi_device, 0, 70 + chordData[4].octave + chordData[4].transpose, 0);  // B flat
				midi_send_noteoff(&midi_device, 0, 71 + chordData[4].octave + chordData[4].transpose, 0);  // B
				midi_send_noteoff(&midi_device, 0, 74 + chordData[4].octave + chordData[4].transpose, 0);  // D note
				midi_send_noteoff(&midi_device, 0, 77 + chordData[4].octave + chordData[4].transpose, 0);  // F
			}
			return false;
		case A_MIN:
			if (record->event.pressed) {
				chordData[5].octave = octave;
				chordData[5].transpose = transpose;

				midi_send_noteon(&midi_device, 0, 69 + chordData[5].octave + chordData[5].transpose, 127);  // A note
				midi_send_noteon(&midi_device, 0, 76 + chordData[5].octave + chordData[5].transpose, 127);  // E note

				if (is_key_8_held) {
					midi_send_noteon(&midi_device, 0, 73 + chordData[5].octave + chordData[5].transpose, 127);  // C# for A major
				} else {
					midi_send_noteon(&midi_device, 0, 72 + chordData[5].octave + chordData[5].transpose, 127);  // C for A minor
				}

				if (is_key_9_held) {
					midi_send_noteon(&midi_device, 0, 79 + chordData[5].octave + chordData[5].transpose, 127);  // G for A minor 7
				}
			} else {
				midi_send_noteoff(&midi_device, 0, 69 + chordData[5].octave + chordData[5].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 72 + chordData[5].octave + chordData[5].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 73 + chordData[5].octave + chordData[5].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 76 + chordData[5].octave + chordData[5].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 79 + chordData[5].octave + chordData[5].transpose, 0);
			}
			return false;

		case B_DIM:
			if (record->event.pressed) {
				chordData[6].octave = octave;
				chordData[6].transpose = transpose;

				midi_send_noteon(&midi_device, 0, 71 + chordData[6].octave + chordData[6].transpose, 127);  // B note
				midi_send_noteon(&midi_device, 0, 74 + chordData[6].octave + chordData[6].transpose, 127);  // D note
				midi_send_noteon(&midi_device, 0, 77 + chordData[6].octave + chordData[6].transpose, 127);  // F note		

				if (is_key_9_held) {
					midi_send_noteon(&midi_device, 0, 81 + chordData[6].octave + chordData[6].transpose, 127);  // A for B half diminished 7
				}
			} else {
				midi_send_noteoff(&midi_device, 0, 71 + chordData[6].octave + chordData[6].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 74 + chordData[6].octave + chordData[6].transpose, 0);				
				midi_send_noteoff(&midi_device, 0, 77 + chordData[6].octave + chordData[6].transpose, 0);
				midi_send_noteoff(&midi_device, 0, 81 + chordData[6].octave + chordData[6].transpose, 0);
			}
			return false;

        case MODWL_UP:
            if (record->event.pressed) {
                if (mod_wheel_val < 127) mod_wheel_val++;
                midi_send_cc(&midi_device, 0, 0x01, mod_wheel_val);
            }
            return false;
        case MODWL_DN:
            if (record->event.pressed) {
                if (mod_wheel_val > 0) mod_wheel_val--;
                midi_send_cc(&midi_device, 0, 0x01, mod_wheel_val);
            }
            return false;
#endif

		default:
            return true;  // Process all other keycodes normally
    }
}