// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

/* Layer indices */
enum layers {
    BASE = 0,
    MOUSE = 1,
    FN = 2,
};

/* ── Key Overrides ────────────────────────────────────────────────────────── */
/*
 * Shift + Backspace = Delete
 * Shift + Esc       = ~ (tilde)
 */
const key_override_t bspc_del_override  = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, KC_DEL);
const key_override_t esc_tilde_override = ko_make_basic(MOD_MASK_SHIFT, KC_ESC,  KC_TILD);

const key_override_t *key_overrides[] = {
    &bspc_del_override,
    &esc_tilde_override,
    NULL,
};

/* ── Keymaps ──────────────────────────────────────────────────────────────── */
/*
 * BASE LAYER (0)
 * ┌────┐    ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┐
 * │Esc │    │ F1 │ F2 │ F3 │ F4 │ │ F5 │ F6 │ F7 │ F8 │ │ F9 │F10 │F11 │F12 │ │PrSc│ScrL│Paus│
 * └────┘    └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┘
 * ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────────┐ ┌────┬────┬────┐ ┌────┬────┬────┬────┐
 * │ `  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │ 9  │ 0  │ -  │ =  │Backspce│ │Ins │Home│PgUp│ │NumL│ /  │ *  │ -  │
 * ├────┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──────┤ ├────┼────┼────┤ ├────┼────┼────┼────┤
 * │Tab   │ Q  │ W  │ E  │ R  │ T  │ Y  │ U  │ I  │ O  │ P  │ [  │ ]  │      │ │Del │End │PgDn│ │ 7  │ 8  │ 9  │    │
 * ├──────┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┐Enter│ └────┴────┴────┘ ├────┼────┼────┤ +  │
 * │CapsLck│ A  │ S  │ D  │ F  │ G  │ H  │ J  │ K  │ L  │ ;  │ '  │ #  │     │                  │ 4  │ 5  │ 6  │    │
 * ├─────┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴────┴─────┤      ┌────┐      ├────┼────┼────┼────┤
 * │Shift│ \  │ Z  │ X  │ C  │ V  │ B  │ N  │ M  │ ,  │ .  │ /  │ Shift      │      │ ↑  │      │ 1  │ 2  │ 3  │    │
 * ├─────┼────┴┬───┴─┬──┴────┴────┴────┴────┴────┴───┬┴────┼────┴┬─────┬─────┤ ┌────┼────┼────┐ ├────┴────┼────┤Entr│
 * │Ctrl │ Win │ Alt │          Space                │ Alt │ Win │ Fn  │ Ctrl│ │ ←  │ ↓  │  → │ │    0    │ .  │    │
 * └─────┴─────┴─────┴───────────────────────────────┴─────┴─────┴─────┴─────┘ └────┴────┴────┘ └─────────┴────┴────┘
 *
 *
 * MOUSE LAYER (1)
 * ┌────┐    ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┐
 * │Esc │    │ F1 │ F2 │ F3 │ F4 │ │ F5 │ F6 │ F7 │ F8 │ │ F9 │F10 │F11 │F12 │ │PrSc│ScrL│Paus│
 * └────┘    └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┘
 * ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────────┐ ┌────┬────┬────┐ ┌────┬────┬────┬────┐
 * │ `  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │ 9  │ 0  │ -  │ =  │Backspce│ │Ins │Home│PgUp│ │Tgl1│LMBt│MMBt│RMBt│
 * ├────┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──────┤ ├────┼────┼────┤ ├────┼────┼────┼────┤
 * │Tab   │ Q  │ W  │ E  │ R  │ T  │ Y  │ U  │ I  │ O  │ P  │ [  │ ]  │      │ │Del │End │PgDn│ │MWh↑│ Ms↑│MBt5│    │
 * ├──────┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┐Enter│ └────┴────┴────┘ ├────┼────┼────┤MBt4│
 * │CapsLck│ A  │ S  │ D  │ F  │ G  │ H  │ J  │ K  │ L  │ ;  │ '  │ #  │     │                  │ Ms→│ Ms↓│ Ms→│    │
 * ├─────┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴────┴─────┤      ┌────┐      ├────┼────┼────┼────┤
 * │Shift│ \  │ Z  │ X  │ C  │ V  │ B  │ N  │ M  │ ,  │ .  │ /  │ Shift      │      │ ↑  │      │MWh↓│MBt7│MBt6│    │
 * ├─────┼────┴┬───┴─┬──┴────┴────┴────┴────┴────┴───┬┴────┼────┴┬─────┬─────┤ ┌────┼────┼────┐ ├────┴────┼────┤Rght│
 * │Ctrl │ Win │ Alt │          Space                │ Alt │ Win │ Fn  │ Ctrl│ │ ←  │ ↓  │  → │ │LeftClick│MClk│Clck│
 * └─────┴─────┴─────┴───────────────────────────────┴─────┴─────┴─────┴─────┘ └────┴────┴────┘ └─────────┴────┴────┘
 *
 *
 * FN LAYER (2) — hold RightMenu/Fn
 * ┌────┐    ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┬────┐ ┌────┬────┬────┐
 * │Slp │    │Calc│Mail│Asst│Brws│ │ F5 │ F6 │ F7 │ F8 │ │ F9 │NoCp│Caps│Boot│ │Rset│RBt │Pwr │
 * └────┘    └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┴────┘ └────┴────┴────┘
 * ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────────┐ ┌────┬────┬────┐ ┌────┬────┬────┬────┐
 * │ `  │ 1  │ 2  │ 3  │ 4  │ 5  │ 6  │ 7  │ 8  │ 9  │ 0  │ -  │ =  │Backspce│ │Ins │Home│Br+ │ │Tgl1│ /  │ *  │ -  │
 * ├────┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──┴─┬──────┤ ├────┼────┼────┤ ├────┼────┼────┼────┤
 * │Tab   │LMBt│ Ms↑│RMBt│MMBt│MWh↑│ Y  │ U  │ I  │ O  │ P  │ [  │ ]  │      │ │Del │End │Br- │ │ 7  │ 8  │ 9  │    │
 * ├──────┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┬───┴┐Play │ └────┴────┴────┘ ├────┼────┼────┤ +  │
 * │1WrdCps│ Ms←│ Ms↓│ Ms→│MWh↑│MWh↓│ H  │ J  │ K  │ L  │ ;  │ '  │Mute│Pause│                  │ 4  │ 5  │ 6  │    │
 * ├─────┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴────┴─────┤      ┌────┐      ├────┼────┼────┼────┤
 * │RMBtn│ \  │ Z  │ X  │MMBt│MWh↓│ B  │NKRO│ M  │ ,  │ .  │ /  │FastForward │      │Vol+│      │ 1  │ 2  │ 3  │    │
 * ├─────┼────┴┬───┴─┬──┴────┴────┴────┴────┴────┴───┬┴────┼────┴┬─────┬─────┤ ┌────┼────┼────┐ ├────┴────┼────┤Entr│
 * │Ctrl │WinLk│RMBtn│        LeftMouseButton        │Menu │WinLk│ Fn  │Rwnd │ │Prvs│Vol-│Next│ │    0    │ .  │    │
 * └─────┴─────┴─────┴───────────────────────────────┴─────┴─────┴─────┴─────┘ └────┴────┴────┘ └─────────┴────┴────┘
 *
 */

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [BASE] = LAYOUT_fullsize_iso_105(
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,  KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,       KC_PSCR, KC_SCRL, KC_PAUS,

        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,   KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC,      KC_INS,  KC_HOME, KC_PGUP,      KC_NUM,  KC_PSLS, KC_PAST, KC_PMNS,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,   KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,               KC_DEL,  KC_END,  KC_PGDN,      KC_P7,   KC_P8,   KC_P9,   KC_PPLS,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,   KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_NUHS, KC_ENT,                                       KC_P4,   KC_P5,   KC_P6,
        KC_LSFT, KC_NUBS, KC_Z,    KC_X,    KC_C,    KC_V,   KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,                        KC_UP,                 KC_P1,   KC_P2,   KC_P3,   KC_PENT,
        KC_LCTL, KC_LGUI, KC_LALT,                           KC_SPC,                             KC_RALT, KC_RGUI, MO(2),   KC_RCTL,      KC_LEFT, KC_DOWN, KC_RGHT,      KC_P0,            KC_PDOT
    ),

    [MOUSE] = LAYOUT_fullsize_iso_105(
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,   KC_F4,  KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,       KC_PSCR, KC_SCRL, KC_PAUS,

        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,   KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC,      KC_INS,  KC_HOME, KC_PGUP,      TG(1),   MS_BTN1, MS_BTN3, MS_BTN2,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,   KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,               KC_DEL,  KC_END,  KC_PGDN,      MS_WHLU, MS_UP,   MS_BTN5, MS_BTN4,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,   KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_NUHS, KC_ENT,                                       MS_LEFT, MS_DOWN, MS_RGHT,
        KC_LSFT, KC_NUBS, KC_Z,    KC_X,    KC_C,    KC_V,   KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,                        KC_UP,                 MS_WHLD, MS_BTN7, MS_BTN6, MS_BTN2,
        KC_LCTL, KC_LGUI, KC_LALT,                           KC_SPC,                             KC_RALT, KC_RGUI, MO(2),   KC_RCTL,      KC_LEFT, KC_DOWN, KC_RGHT,      MS_BTN1,          MS_BTN3
    ),


    [FN] = LAYOUT_fullsize_iso_105(
        KC_SLEP,          KC_CALC, KC_ASST, KC_MAIL, KC_WHOM, KC_F5,   KC_F6,   KC_F7,  KC_F8,   KC_F9,   CL_CTRL, CL_CAPS, QK_BOOT,      EE_CLR,  QK_RBT,  KC_PWR,

        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,   KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC,      KC_INS,  KC_HOME, KC_BRIU,      TG(1),   KC_PSLS, KC_PAST, KC_PMNS,
        KC_TAB,  MS_BTN1, MS_UP,   MS_BTN2, MS_BTN3, MS_WHLU, KC_Y,    KC_U,    KC_I,   KC_O,    KC_P,    KC_LBRC, KC_RBRC,               KC_DEL,  KC_END,  KC_BRID,      KC_P7,   KC_P8,   KC_P9,   KC_PPLS,
        CW_TOGG, MS_LEFT, MS_DOWN, MS_RGHT, MS_WHLU, MS_WHLD, KC_H,    KC_J,    KC_K,   KC_L,    KC_SCLN, KC_QUOT, KC_MUTE, KC_MPLY,                                      KC_P4,   KC_P5,   KC_P6,
        MS_BTN2, KC_NUBS, KC_Z,    KC_X,    MS_BTN3, MS_WHLD, KC_B,    NK_TOGG, KC_M,   KC_COMM, KC_DOT,  KC_SLSH, KC_MFFD,                        KC_VOLU,               KC_P1,   KC_P2,   KC_P3,   KC_PENT,
        KC_LCTL, GU_TOGG, MS_BTN2,                            MS_BTN1,                           KC_APP,  GU_TOGG, MO(2),   KC_MRWD,      KC_MPRV, KC_VOLD, KC_MNXT,      KC_P0,            KC_PDOT
    ),

};
