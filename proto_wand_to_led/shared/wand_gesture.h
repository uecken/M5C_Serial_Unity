#pragma once
// ============================================================
// proto_wand_to_led / shared / wand_gesture.h
//   杖のジェスチャ判定コア (重力基準フリック検出 + Wingardium 浮遊)。
//   M5StickC(ESP32) / XIAO nRF52840 Sense など全プラットフォーム共通。
//
//   プラットフォーム依存 (BLE 送信 / LED フラッシュ) は関数ポインタで注入する:
//     emit(trigger_id, strength, target_id) … beacon 送信 (NimBLE / Bluefruit)
//     feedback()                            … 検出時の手元 LED フラッシュ
//   軸マウントと閾値は Config で受け取る (device_config.h の WAND_FORWARD/RIGHT を渡す)。
//
//   ※ 各ボードは単一の main.cpp TU から 1 個だけ生成する想定。
// ============================================================
#include <Arduino.h>
#include <math.h>
#include "beacon_protocol.h"

namespace wand_gesture {

// プラットフォーム注入用コールバック
using EmitFn     = void (*)(uint8_t trigger_id, uint8_t strength, uint16_t target_id);
using FeedbackFn = void (*)();

// 実行時チューニング可能なパラメータ (各ボードが NVS/LittleFS で永続化)
//   符号付き軸コード: +X=1 +Y=2 +Z=3 (負で反転)。device_config.h の WAND_* を入れる。
struct Config {
  float    flick_threshold_g = 1.5f;   // linear accel 閾値 [g]
  float    updown_ratio      = 0.75f;  // |方向成分|/|動き| の判定比
  uint32_t cooldown_ms       = 1000;   // トリガ間隔 [ms]
  float    grav_alpha        = 0.02f;  // 重力推定 EMA 係数 (小=ゆっくり)
  float    still_band        = 0.15f;  // 静止判定: ||accel|-1g| がこれ未満なら静止 [g]
  int      forward_axis      = 2;      // 杖先端 (前方向) の軸コード = +Y
  int      right_axis        = 1;      // 右方向の軸コード = +X
};

// ============================================================
// ジェスチャ検出器
//   重力ベクトルを「上」基準として振り上げ/下げ/前突き/横振りを分類:
//     上振り → LUMOS / 下振り → NOX / 前突き(+forward) → EXPECTO / 横振り → INCENDIO
//     どれにも当てはまらない強い振りは暫定 LUMOS にフォールバック (無反応にしない)。
//   Wingardium Leviosa: 上向き保持を継続で浮遊モード → ピッチを 20Hz 連続送信。
// ============================================================
struct Detector {
  Config     cfg;
  EmitFn     emit     = nullptr;
  FeedbackFn feedback = nullptr;

  // --- 状態 ---
  uint32_t last_trigger_ms = 0;
  float    grav_x = 0.0f, grav_y = 0.0f, grav_z = 1.0f;  // 重力推定 (EMA)
  int      still_count    = 0;
  bool     ready          = false;
  uint32_t last_motion_ms = 0;   // 最後に「静止でない」状態だった時刻 (sleep 判定用)

  uint32_t up_hold_since   = 0;  // 上向き保持の開始時刻 (0=保持なし)
  uint32_t levitation_until = 0; // 浮遊モード終了時刻 (0=非浮遊)
  uint32_t levit_next_send  = 0;

  // --- 定数 ---
  static constexpr int      READY_STILL_SAMPLES   = 25;     // 約 0.25s (100Hz)
  static constexpr float    WINGARDIUM_PITCH_SIN   = 0.70f; // 先端が上 ~45° 以上で活性化候補
  static constexpr float    WINGARDIUM_STILL_LMAG  = 0.30f; // 動き成分がこれ未満 = 静止
  static constexpr uint32_t WINGARDIUM_HOLD_MS     = 800;   // 上向き保持の活性化時間
  static constexpr uint32_t LEVITATION_MS          = 8000;  // 浮遊モード持続
  static constexpr uint32_t LEVIT_SEND_INTERVAL_MS = 50;    // ピッチ送信間隔 = 20Hz

  // プラットフォーム hook を注入 (cfg は別途 gcfg/cfg 永続化層が設定)
  void begin(EmitFn e, FeedbackFn f) { emit = e; feedback = f; }

  // 起動直後の静止姿勢で重力初期値をセット
  void set_gravity(float ax, float ay, float az) { grav_x = ax; grav_y = ay; grav_z = az; }

  bool is_levitating() const { return levitation_until != 0; }

  // 符号付き軸コード (1=+X,2=+Y,3=+Z, 負で反転) で linear[3] を射影
  static inline float project_axis(float lx, float ly, float lz, int axis_code) {
    int   idx = (axis_code < 0 ? -axis_code : axis_code) - 1;   // 0,1,2
    float v   = (idx == 0) ? lx : (idx == 1) ? ly : lz;
    return (axis_code > 0) ? v : -v;
  }

  // |accel|≈1g の静止時のみ重力を更新 (フリック混入を防ぐ)。
  void update_gravity(float ax, float ay, float az) {
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(a_mag - 1.0f) < cfg.still_band) {
      grav_x += cfg.grav_alpha * (ax - grav_x);
      grav_y += cfg.grav_alpha * (ay - grav_y);
      grav_z += cfg.grav_alpha * (az - grav_z);
      if (still_count < READY_STILL_SAMPLES) still_count++;
      if (still_count >= READY_STILL_SAMPLES) ready = true;
    } else {
      last_motion_ms = millis();   // 動き中 → 重力凍結。ready 維持。sleep 防止
    }
  }

  void check(float ax, float ay, float az) {
    uint32_t now = millis();

    // 重力方向 (= 上方向の単位ベクトル)
    float gmag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    if (gmag < 0.1f) gmag = 0.1f;
    float ux = grav_x / gmag, uy = grav_y / gmag, uz = grav_z / gmag;

    // linear accel = 生 − 重力
    float lx = ax - grav_x, ly = ay - grav_y, lz = az - grav_z;
    float lmag = sqrtf(lx*lx + ly*ly + lz*lz);

    if (lmag < cfg.flick_threshold_g || (now - last_trigger_ms) <= cfg.cooldown_ms) {
      return;  // 動きが弱い or クールダウン中
    }

    // 各方向への射影
    float up_proj    = lx*ux + ly*uy + lz*uz;                       // 鉛直 (重力フレーム)
    float fwd_proj   = project_axis(lx, ly, lz, cfg.forward_axis);  // 前後 (機体軸)
    float right_proj = project_axis(lx, ly, lz, cfg.right_axis);    // 左右 (機体軸)
    float ratio_up    = fabsf(up_proj)    / lmag;
    float ratio_fwd   = fabsf(fwd_proj)   / lmag;
    float ratio_right = fabsf(right_proj) / lmag;

    // strength: linear accel の強さを 0-255 にマップ
    int s = (int)((lmag - cfg.flick_threshold_g) * 91.0f);
    if (s < 0) s = 0; if (s > 255) s = 255;

    // 判定優先: 上下 (重力) > 前突き (機体前) > 横振り (機体左右)。
    //   明確な呪文に当てはまらない振りは暫定 LUMOS にフォールバック (無反応にしない)。
    uint8_t trig;
    const char* name;
    if (ratio_up >= cfg.updown_ratio && up_proj < 0) {
      trig = wand_beacon::TRIG_NOX;              name = "NOX (down)";
    } else if (ratio_fwd >= cfg.updown_ratio && fwd_proj > 0) {
      trig = wand_beacon::TRIG_EXPECTO_PATRONUM; name = "EXPECTO PATRONUM (thrust fwd)";
    } else if (ratio_right >= cfg.updown_ratio) {
      trig = wand_beacon::TRIG_INCENDIO;         name = "INCENDIO (side-swing)";
    } else if (ratio_up >= cfg.updown_ratio && up_proj > 0) {
      trig = wand_beacon::TRIG_LUMOS;            name = "LUMOS (up)";
    } else {
      trig = wand_beacon::TRIG_LUMOS;            name = "LUMOS (fallback/shake)";
    }

    if (emit) emit(trig, (uint8_t)s, wand_beacon::TARGET_ALL);  // 全機宛て
    last_trigger_ms = now;
    if (feedback) feedback();   // 手元 LED を一瞬光らせて検出をフィードバック
    Serial.printf("*** %s  lmag=%.2fg up=%.2f(%.2f) fwd=%.2f(%.2f) rt=%.2f(%.2f) s=%d ***\n",
                  name, lmag, up_proj, ratio_up, fwd_proj, ratio_fwd, right_proj, ratio_right, s);
  }

  // 浮遊モードを強制開始 (シリアル 'w' のベンチテスト用)
  void force_levitation() {
    levitation_until = millis() + LEVITATION_MS;
    levit_next_send  = millis();
    up_hold_since    = 0;
    last_trigger_ms  = millis();
    Serial.println("[WINGARDIUM] force levitation (serial 'w')");
  }

  // 毎ループ呼ぶ: 上向き保持で活性化 → 浮遊モード中はピッチを連続送信
  void poll_wingardium(float ax, float ay, float az) {
    uint32_t now = millis();
    float gmag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    if (gmag < 0.1f) gmag = 0.1f;
    float ux = grav_x/gmag, uy = grav_y/gmag, uz = grav_z/gmag;

    // ピッチ: 先端(forward)が上方向にどれだけ向いているか (-1..+1)
    float pitch_sin = project_axis(ux, uy, uz, cfg.forward_axis);
    float lx = ax-grav_x, ly = ay-grav_y, lz = az-grav_z;
    float lmag = sqrtf(lx*lx + ly*ly + lz*lz);

    // --- 浮遊モード中: ピッチを連続送信 ---
    if (levitation_until != 0) {
      if ((int32_t)(now - levitation_until) >= 0) {
        levitation_until = 0;            // タイムアウト → 浮遊終了
        Serial.println("[WINGARDIUM] levitation end");
        return;
      }
      if ((int32_t)(now - levit_next_send) >= 0) {
        int b = (int)((pitch_sin + 1.0f) * 127.5f);   // -1..+1 → 0..255
        if (b < 0) b = 0; if (b > 255) b = 255;
        if (emit) emit(wand_beacon::TRIG_WINGARDIUM, (uint8_t)b, wand_beacon::TARGET_ALL);
        levit_next_send = now + LEVIT_SEND_INTERVAL_MS;
        last_motion_ms  = now;           // sleep 防止
      }
      return;  // 浮遊中は活性化判定しない
    }

    // --- 活性化判定: 上向き + 静止を継続 ---
    if (pitch_sin > WINGARDIUM_PITCH_SIN && lmag < WINGARDIUM_STILL_LMAG) {
      if (up_hold_since == 0) up_hold_since = now;
      if (now - up_hold_since >= WINGARDIUM_HOLD_MS) {
        levitation_until = now + LEVITATION_MS;   // 浮遊モード開始
        levit_next_send  = now;
        up_hold_since    = 0;
        last_trigger_ms  = now;                   // 他ジェスチャのクールダウンと共有
        Serial.println("[WINGARDIUM] activate \xe2\x86\x92 levitation");
      }
    } else {
      up_hold_since = 0;  // 条件を外れたらリセット
    }
  }
};

}  // namespace wand_gesture
