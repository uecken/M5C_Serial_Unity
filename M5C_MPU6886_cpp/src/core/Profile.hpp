// Burst Motion - core/Profile.hpp
// LittleFS にプロファイル (ActionRule の集合) を JSON 保存/読込
// 形式: /profiles/<name>.json
//
// {
//   "schema_version": 1,
//   "name": "...",
//   "rules": [
//     {
//       "id": 1, "name": "shake_a", "ui_mode": "oneshot",
//       "accel_abs_threshold": 2.5, "key": "a", "cooldown_ms": 500,
//       ...
//     }, ...
//   ]
// }
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "types.hpp"
#include <vector>
#include <string>

namespace BurstMotion {

class Profile {
public:
    static constexpr const char* PROFILES_DIR = "/profiles";
    static constexpr const char* ACTIVE_PROFILE_FILE = "/active_profile.txt";

    static bool init() {
        if (!LittleFS.begin(true)) {
            return false;
        }
        // /profiles ディレクトリ作成
        if (!LittleFS.exists(PROFILES_DIR)) {
            LittleFS.mkdir(PROFILES_DIR);
        }
        return true;
    }

    // プロファイル一覧
    static std::vector<std::string> list() {
        std::vector<std::string> names;
        File dir = LittleFS.open(PROFILES_DIR);
        if (!dir || !dir.isDirectory()) return names;
        File f = dir.openNextFile();
        while (f) {
            String name = f.name();
            // ファイル名のみ取得 (パスを除去)
            int slash = name.lastIndexOf('/');
            if (slash >= 0) name = name.substring(slash + 1);
            if (name.endsWith(".json")) {
                name = name.substring(0, name.length() - 5);  // remove .json
                names.push_back(std::string(name.c_str()));
            }
            f = dir.openNextFile();
        }
        return names;
    }

    // ルール群を JSON 化して保存
    static bool save(const char* name, const std::vector<ActionRule>& rules, JsonDocument& errorOut) {
        String path = String(PROFILES_DIR) + "/" + name + ".json";

        JsonDocument doc;
        doc["schema_version"] = 1;
        doc["name"] = name;
        JsonArray arr = doc["rules"].to<JsonArray>();

        for (const auto& r : rules) {
            JsonObject ro = arr.add<JsonObject>();
            ro["id"] = r.id;
            ro["name"] = r.name;
            // 各 state を保存
            JsonArray states = ro["states"].to<JsonArray>();
            for (uint8_t i = 0; i < r.states_count; i++) {
                JsonObject so = states.add<JsonObject>();
                serializeState(so, r.states[i]);
            }
            ro["loop"] = r.loop;
            ro["priority"] = r.priority;
            ro["cooldown_ms"] = r.cooldown_ms;
            // Phase 5.39: posture_basis 保存 (default 0 = absolute、後方互換)
            ro["posture_basis"] = r.posture_basis;
            // Phase 5.39.3a: waypoint_order 保存 (default 0 = sequential、後方互換)
            ro["waypoint_order"] = r.waypoint_order;
        }

        File f = LittleFS.open(path, "w");
        if (!f) {
            errorOut["err"] = "open_write_failed";
            return false;
        }
        size_t written = serializeJson(doc, f);
        f.close();
        if (written == 0) {
            errorOut["err"] = "serialize_failed";
            return false;
        }
        return true;
    }

    // JSON 読込 → ActionRule に展開
    static bool load(const char* name, std::vector<ActionRule>& outRules, JsonDocument& errorOut) {
        String path = String(PROFILES_DIR) + "/" + name + ".json";
        if (!LittleFS.exists(path)) {
            errorOut["err"] = "not_found";
            return false;
        }
        File f = LittleFS.open(path, "r");
        if (!f) {
            errorOut["err"] = "open_failed";
            return false;
        }
        JsonDocument doc;
        DeserializationError de = deserializeJson(doc, f);
        f.close();
        if (de) {
            errorOut["err"] = "parse_error";
            errorOut["detail"] = de.c_str();
            return false;
        }

        outRules.clear();
        JsonArray arr = doc["rules"];
        for (JsonObject ro : arr) {
            ActionRule r = {};
            r.id = ro["id"] | 0;
            const char* nm = ro["name"] | "rule";
            strncpy(r.name, nm, sizeof(r.name) - 1);
            r.loop = ro["loop"] | false;
            r.priority = ro["priority"] | 0;
            r.cooldown_ms = ro["cooldown_ms"] | 500;
            // Phase 5.39: posture_basis 読込 (省略時 0 = absolute、後方互換)
            r.posture_basis = (uint8_t)(ro["posture_basis"] | 0);
            // Phase 5.39.3a: waypoint_order 読込 (省略時 0 = sequential、後方互換)
            r.waypoint_order = (uint8_t)(ro["waypoint_order"] | 0);
            JsonArray states = ro["states"];
            uint8_t i = 0;
            for (JsonObject so : states) {
                if (i >= 4) break;
                deserializeState(so, r.states[i]);
                i++;
            }
            r.states_count = i;
            r.current_state = -1;
            r.state_enter_ms = 0;
            r.last_fire_ms = 0;
            // Phase 5.39 runtime 初期化
            r.q_ref[0] = 1.0f; r.q_ref[1] = 0.0f; r.q_ref[2] = 0.0f; r.q_ref[3] = 0.0f;
            r.q_ref_valid = false;
            r.stillness_since_ms = 0;
            outRules.push_back(r);
        }
        return true;
    }

    static bool remove(const char* name, JsonDocument& errorOut) {
        String path = String(PROFILES_DIR) + "/" + name + ".json";
        if (!LittleFS.exists(path)) {
            errorOut["err"] = "not_found";
            return false;
        }
        if (!LittleFS.remove(path)) {
            errorOut["err"] = "remove_failed";
            return false;
        }
        return true;
    }

    static bool setActive(const char* name) {
        File f = LittleFS.open(ACTIVE_PROFILE_FILE, "w");
        if (!f) return false;
        f.print(name);
        f.close();
        return true;
    }

    static String getActive() {
        if (!LittleFS.exists(ACTIVE_PROFILE_FILE)) return "";
        File f = LittleFS.open(ACTIVE_PROFILE_FILE, "r");
        if (!f) return "";
        String name = f.readString();
        f.close();
        name.trim();
        return name;
    }

private:
    static void serializeState(JsonObject& out, const State& s) {
        // 簡易シリアライズ: condition + on_enter / on_exit のキー要素のみ
        JsonObject c = out["match_condition"].to<JsonObject>();
        c["logic_op"] = (uint8_t)s.match_condition.logic_op;
        // accel
        if (s.match_condition.accel.enabled) {
            JsonObject a = c["accel"].to<JsonObject>();
            a["abs_threshold"] = s.match_condition.accel.abs_threshold;
            a["use_per_axis"] = s.match_condition.accel.use_per_axis;
            a["comparison"] = (uint8_t)s.match_condition.accel.comparison;
        }
        // posture (Phase 5.21: quat / quat_dot_min 追加で 3D 表示・closest 計算が NVS 復元後も正確)
        if (s.match_condition.posture.enabled) {
            JsonObject p = c["posture"].to<JsonObject>();
            p["judge_by"] = (uint8_t)s.match_condition.posture.judge_by;
            JsonArray e = p["euler"].to<JsonArray>();
            for (int i = 0; i < 3; i++) e.add(s.match_condition.posture.euler[i]);
            JsonArray et = p["euler_tol"].to<JsonArray>();
            for (int i = 0; i < 3; i++) et.add(s.match_condition.posture.euler_tol[i]);
            JsonArray q = p["quat"].to<JsonArray>();
            for (int i = 0; i < 4; i++) q.add(s.match_condition.posture.quat[i]);
            p["quat_dot_min"] = s.match_condition.posture.quat_dot_min;
        }
        // button
        if (s.match_condition.button.enabled) {
            JsonObject b = c["button"].to<JsonObject>();
            b["idx"] = s.match_condition.button.idx;
            b["state"] = s.match_condition.button.state;
        }

        // Phase 5.39: stillness (静止検出) — required=false の場合は serialize 省略 (省サイズ)
        if (s.match_condition.stillness_required) {
            JsonObject st = c["stillness"].to<JsonObject>();
            st["required"] = true;
            st["window_ms"] = s.match_condition.stillness_window_ms;
            st["accel_th_mg"] = s.match_condition.stillness_accel_th_mg;
            st["gyro_th_dps"] = s.match_condition.stillness_gyro_th_dps;
        }

        // on_enter
        JsonObject oe = out["on_enter"].to<JsonObject>();
        oe["type"] = (uint8_t)s.on_enter.type;
        oe["keys_len"] = s.on_enter.keys_len;
        JsonArray k = oe["keys"].to<JsonArray>();
        for (uint8_t i = 0; i < s.on_enter.keys_len && i < 24; i++) k.add(s.on_enter.keys[i]);
        // Phase 5.14: key_modes (FIRE_MACRO 同時押し対応用)
        JsonArray km = oe["key_modes"].to<JsonArray>();
        for (uint8_t i = 0; i < s.on_enter.keys_len && i < 24; i++) km.add(s.on_enter.key_modes[i]);
        oe["modifiers"] = s.on_enter.modifiers;
        oe["duration_ms"] = s.on_enter.duration_ms;
        oe["interval_ms"] = s.on_enter.interval_ms;

        // on_exit
        JsonObject ox = out["on_exit"].to<JsonObject>();
        ox["type"] = (uint8_t)s.on_exit.type;
        ox["keys_len"] = s.on_exit.keys_len;
        JsonArray kx = ox["keys"].to<JsonArray>();
        for (uint8_t i = 0; i < s.on_exit.keys_len && i < 24; i++) kx.add(s.on_exit.keys[i]);

        out["min_dwell_ms"] = s.min_dwell_ms;
        out["max_dwell_ms"] = s.max_dwell_ms;
    }

    static void deserializeState(JsonObject& in, State& s) {
        memset(&s, 0, sizeof(s));
        JsonObject c = in["match_condition"];
        s.match_condition.logic_op = (LogicOp)(uint8_t)(c["logic_op"] | 0);
        if (c["accel"].is<JsonObject>()) {
            JsonObject a = c["accel"];
            s.match_condition.accel.enabled = true;
            s.match_condition.accel.abs_threshold = a["abs_threshold"] | 0.0f;
            s.match_condition.accel.use_per_axis = a["use_per_axis"] | false;
            s.match_condition.accel.comparison = (Comparison)(uint8_t)(a["comparison"] | 0);
        }
        if (c["posture"].is<JsonObject>()) {
            JsonObject p = c["posture"];
            s.match_condition.posture.enabled = true;
            s.match_condition.posture.judge_by = (PostureJudge)(uint8_t)(p["judge_by"] | 0);
            JsonArray e = p["euler"];
            JsonArray et = p["euler_tol"];
            for (int i = 0; i < 3; i++) {
                s.match_condition.posture.euler[i] = e ? e[i].as<float>() : 0.0f;
                s.match_condition.posture.euler_tol[i] = et ? et[i].as<float>() : 180.0f;
            }
            // Phase 5.21: quat / quat_dot_min も復元 (Web 3D 表示と Closest BY_QUAT モードのため)
            JsonArray q = p["quat"];
            for (int i = 0; i < 4; i++) {
                s.match_condition.posture.quat[i] = q ? q[i].as<float>() : (i == 0 ? 1.0f : 0.0f);
            }
            s.match_condition.posture.quat_dot_min = p["quat_dot_min"] | 0.95f;
        }
        if (c["button"].is<JsonObject>()) {
            JsonObject b = c["button"];
            s.match_condition.button.enabled = true;
            s.match_condition.button.idx = b["idx"] | 0;
            s.match_condition.button.state = b["state"] | 0;
        }

        // Phase 5.39: stillness 読込 (省略時 required=false で後方互換)
        if (c["stillness"].is<JsonObject>()) {
            JsonObject st = c["stillness"];
            s.match_condition.stillness_required = st["required"] | false;
            s.match_condition.stillness_window_ms = st["window_ms"] | 200;
            s.match_condition.stillness_accel_th_mg = (uint8_t)(st["accel_th_mg"] | 100);
            s.match_condition.stillness_gyro_th_dps = (uint8_t)(st["gyro_th_dps"] | 5);
        }

        JsonObject oe = in["on_enter"];
        s.on_enter.type = (ActionType)(uint8_t)(oe["type"] | 0);
        s.on_enter.keys_len = oe["keys_len"] | 0;
        JsonArray k = oe["keys"];
        for (uint8_t i = 0; i < s.on_enter.keys_len && i < 24 && k; i++) {
            s.on_enter.keys[i] = (uint8_t)(k[i] | 0);
        }
        // Phase 5.14: key_modes (旧プロファイルでは存在しない場合があるので fallback)
        JsonArray km = oe["key_modes"];
        for (uint8_t i = 0; i < s.on_enter.keys_len && i < 24; i++) {
            s.on_enter.key_modes[i] = km ? (uint8_t)(km[i] | 0) : 0;
        }
        s.on_enter.modifiers = oe["modifiers"] | 0;
        s.on_enter.duration_ms = oe["duration_ms"] | 30;
        s.on_enter.interval_ms = oe["interval_ms"] | 30;

        JsonObject ox = in["on_exit"];
        s.on_exit.type = (ActionType)(uint8_t)(ox["type"] | 0);
        s.on_exit.keys_len = ox["keys_len"] | 0;
        JsonArray kx = ox["keys"];
        for (uint8_t i = 0; i < s.on_exit.keys_len && i < 24 && kx; i++) {
            s.on_exit.keys[i] = (uint8_t)(kx[i] | 0);
        }

        s.min_dwell_ms = in["min_dwell_ms"] | 0;
        s.max_dwell_ms = in["max_dwell_ms"] | 0;
    }
};

}  // namespace BurstMotion
