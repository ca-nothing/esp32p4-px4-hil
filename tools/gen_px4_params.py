#!/usr/bin/env python3
"""PX4 kaynak agacindaki TUM parametreleri (yaml) tarayip ESP32-P4 FC icin
gomulu MAVLink param tablosu uretir: main/fc_px4_params.{c,h}.

Kullanim:
    python tools/gen_px4_params.py [PX4_SRC_DIR]
    (varsayilan PX4_SRC_DIR = C:/_Px4/PX4-Autopilot/src)

Bagimlilik: pyyaml  (pip install pyyaml)

Not: QGC, MAV_AUTOPILOT_PX4 araclar icin tum PX4 parametrelerinin var olmasini
bekler (her Vehicle Config sayfasi kendi paramlarini sorar -> eksikse "Missing
params" diyalogu). Bu script o tam seti gomer. Degerler PX4 varsayilanlari;
FC davranisina baglanma firmware tarafinda ayri yapilir (Faz 3)."""
import os
import sys
# NOT: yaml SADECE yaml-tarama yolu (collect) icin gerekli; --from-existing icin
# import edilmez (pyyaml yokken de px4_parameters.hpp uretilebilsin).

PX4_SRC = sys.argv[1] if len(sys.argv) > 1 else "C:/_Px4/PX4-Autopilot/src"
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "main")
# Deger override'lari (PX4 yaml varsayilani QGC HIL icin uygunsuzsa). Full-regen
# yolunda uygulanir -> elle fc_px4_params.c duzenlemesi REGEN'DE KAYBOLMAZ.
#  - COM_RC_IN_MODE: radio kalibrasyonunu atlat (joystick/RC yok).
#  - CAL_*0_ID / _PRIO: HIL sim-sensor kalibrasyon kimlikleri. Commander '-h' (HIL)
#    PREFLIGHT_CALIBRATION'i reddeder (Commander.cpp:1434) -> kullanici QGC'den kalibre
#    EDEMEZ. PX4 SITL ayni isi rcS:139-149'da yapar (CAL_ACC0/GYRO0/MAG0_ID on-yukler).
#    Degerler fc_px4_adapter.cpp:378 device_id'leri; calibration::FindCurrentCalibration-
#    Index (Utilities.cpp:59) CAL_xxx0_ID==device_id TAM esler. PRIO=50=DEFAULT_PRIORITY
#    (acik yaziyoruz: device_id'ler I2C bus_type decode -> -1 fallback'i 75/external olabilir).
OVERRIDES = {
    "COM_RC_IN_MODE": 1.0,
    "CAL_ACC0_ID": 4609.0,  "CAL_ACC0_PRIO": 50.0,   # 0x1201 ACCEL_ID
    "CAL_GYRO0_ID": 4353.0, "CAL_GYRO0_PRIO": 50.0,  # 0x1101 GYRO_ID
    "CAL_MAG0_ID": 4865.0,  "CAL_MAG0_PRIO": 50.0,   # 0x1301 MAG_ID
    "CAL_BARO0_ID": 5121.0, "CAL_BARO0_PRIO": 50.0,  # 0x1401 BARO_ID
}


def to_float(v):
    try:
        if isinstance(v, bool):
            return 1.0 if v else 0.0
        return float(v)
    except Exception:
        return 0.0


def map_type(t):  # MAV_PARAM_TYPE: REAL32=9, INT32=6
    return 9 if (t or "").lower() == "float" else 6


def float_literal(v):
    """Gecerli C++ float-literal uret. %.7g whole-number'da '2' verir -> '2f' GECERSIZ
    (float-suffix nokta/us ister) + narrowing-init icin 'f' SART (param_value_u.f = float).
    Cozum: nokta/us yoksa '.0' ekle -> '2.0f'. NaN/Inf -> makro."""
    import math
    if math.isnan(v):
        return "NAN"
    if math.isinf(v):
        return "INFINITY" if v > 0 else "(-INFINITY)"
    s = "%.7g" % v
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def collect(src):
    import yaml  # lazy: yalniz yaml-tarama yolunda gerekli
    params = {}
    files = 0
    for root, _, names in os.walk(src):
        for n in names:
            if not n.endswith(".yaml"):
                continue
            try:
                with open(os.path.join(root, n), "r", encoding="utf-8") as f:
                    doc = yaml.safe_load(f)
            except Exception:
                continue
            if not isinstance(doc, dict):
                continue
            groups = doc.get("parameters")
            if not isinstance(groups, list):
                continue
            files += 1
            for g in groups:
                defs = (g or {}).get("definitions") or {}
                for raw, d in defs.items():
                    if not isinstance(d, dict):
                        continue
                    mt = map_type(d.get("type"))
                    default = d.get("default", 0)
                    if "${i}" in raw:
                        start = int(d.get("instance_start", 0))
                        count = int(d.get("num_instances", 1))
                        for idx in range(start, start + count):
                            nm = raw.replace("${i}", str(idx))
                            dv = (default[idx - start]
                                  if isinstance(default, list) and (idx - start) < len(default)
                                  else (0 if isinstance(default, list) else default))
                            params[nm] = (mt, to_float(dv))
                    else:
                        params[raw] = (mt, to_float(default))
    return params, files


def write_px4_parameters_hpp(items, out_path, source_desc):
    """GERCEK PX4 px4_parameters.hpp uretir (src/lib/parameters/templates/px4_parameters.hpp.jinja
    FORMATI, BIREBIR): px4::params enum + param_info_s parameters[] (.name + .val{.f/.i}) +
    parameters_type[] + parameters_volatile[] + parameters_readonly[].

    parameters.cpp + parameters_common.cpp + ConstLayer.h BU dosyadaki 'param_info_s parameters[]'
    dizisine BAGIMLI (firmware-default degerleri). Eski kismi-header bu diziyi VERMIYORDU -> gercek
    backend derlenemiyordu = ana step-3 blokeri. KRITIK: enum sirasi = items sirasi (alfabetik) =
    parameters[]/parameters_type[] indeksi; px4::params::X enum-degeri parameters[X] ile hizali olmali.
    MAV type 9(REAL32)->FLOAT, 6(INT32)->INT32.
    NOT (rule-7 acik-fark): volatile/readonly metadata fc_px4_params.c'de yok -> BOS diziler. Gercek
    PX4 da readonly'yi default'ta BOS uretir (readonly_config.py None->set()); bos dizi GCC-uzantisi,
    parameters_common.cpp:89/104 range-for 0-tur eder. volatile-bos = birkac param 'saklanmaz' yerine
    'saklanir' (kucuk sapma; gerekirse yaml-taramayla zenginlestirilebilir)."""
    # BINARY-SEARCH SARTI (parameters.cpp:182 param_find_internal): parameters[] alfabetik SIRALI olmali.
    # fc_px4_params.c'de el-ekli HIL_ACT_FUNC* SONDA -> burada TUM listeyi yeniden sirala. enum+parameters[]+
    # parameters_type[] AYNI sirada kalir -> px4::params::X indeksi parameters[X] ile hizali; modul Param<>
    # referanslari ad'a gore cozulur (sira-degisimi guvenli, indeks-icsel-tutarlilik korunur).
    items = sorted(items, key=lambda it: it[0])
    N = len(items)
    out = [
        "/* OTOMATIK URETILDI - tools/gen_px4_params.py. GERCEK PX4 autogen muadili",
        " * (src/lib/parameters/templates/px4_parameters.hpp.jinja FORMATI). Kaynak: %s (%d param)." % (source_desc, N),
        " * parameters.cpp + parameters_common.cpp + ConstLayer.h BU dosyadaki param_info_s parameters[]",
        " * dizisine BAGIMLI (firmware-default). ELLE DUZENLEME - script'i tekrar calistir. */",
        "#pragma once",
        "",
        "#include <math.h>   // NAN",
        "#include <stdint.h>",
        "#include <parameters/param.h>",
        "",
        "namespace px4 {",
        "",
        "/// Enum with all parameters (alfabetik; enum 0'dan baslar +1 -> parameters[] indeksi).",
        "enum class params : uint16_t {",
    ]
    for it in items:
        out.append("\t%s," % it[0])
    out.append("};")
    out.append("")
    out.append("static constexpr param_info_s parameters[] = {")
    for it in items:
        name, t = it[0], it[1]
        v = it[2] if len(it) > 2 else 0.0
        if t == 9:   # REAL32 -> FLOAT
            out.append('\t{ .name = "%s", .val = { .f = %s } },' % (name, float_literal(v)))
        else:        # INT32
            out.append('\t{ .name = "%s", .val = { .i = %d } },' % (name, int(round(v))))
    out.append("};")
    out.append("")
    out.append("static constexpr param_type_t parameters_type[] = {")
    for it in items:
        # MAV_PARAM_TYPE: 9=REAL32 -> PARAM_TYPE_FLOAT ; 6=INT32 -> PARAM_TYPE_INT32
        out.append("\tPARAM_TYPE_%s," % ("FLOAT" if it[1] == 9 else "INT32"))
    out.append("};")
    out.append("")
    out.append("// rule-7 acik-fark: volatile/readonly metadata fc_px4_params.c'de yok -> BOS")
    out.append("// (gercek PX4 readonly de readonly_config-None'da BOS uretir; bos dizi GCC-uzantisi).")
    out.append("static constexpr params parameters_volatile[] = {};")
    out.append("static constexpr params parameters_readonly[] = {};")
    out.append("")
    out.append("} // namespace px4")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("wrote %s (%d params: enum + parameters[] + type[] + volatile/readonly)" % (os.path.normpath(out_path), N))


def parse_existing_c(c_path):
    """Mevcut (commit'li) fc_px4_params.c'yi ayristirir -> [(name, mav_type, default)] SIRAYLA.
    Bu, deponun GERCEK indeks sirasidir (yaml'dan yeniden turetme drift riski yok).
    3. alan (def_val) gercek backend ConstLayer'in firmware-default'u (param_info_s.val) icin SART."""
    import re
    items = []
    rx = re.compile(r'^\s*\{\s*"([^"]+)"\s*,\s*(\d+)\s*,\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)f?\s*\}')
    with open(c_path, "r", encoding="utf-8") as f:
        for line in f:
            m = rx.match(line)
            if m:
                items.append((m.group(1), int(m.group(2)), float(m.group(3))))
    return items


def main():
    # --from-existing: mevcut fc_px4_params.c'den SADECE px4_parameters.hpp uret
    # (depoyu/yaml'i ELLEME; index-eslesme yapisal garanti).
    if "--from-existing" in sys.argv:
        c_path = os.path.join(OUT_DIR, "fc_px4_params.c")
        items = parse_existing_c(c_path)
        hpp = os.path.join(OUT_DIR, "PX4", "src", "lib", "parameters", "px4_parameters.hpp")
        write_px4_parameters_hpp(items, hpp, os.path.normpath(c_path))
        return

    params, files = collect(PX4_SRC)
    items = [(k, v) for k, v in sorted(params.items()) if len(k) <= 16]
    N = len(items)
    print("yaml_files=%d params=%d (<=16char emitted=%d)" % (files, len(params), N))

    h = OUT_DIR and os.path.join(OUT_DIR, "fc_px4_params.h")
    c = os.path.join(OUT_DIR, "fc_px4_params.c")

    hdr = [
        "/* OTOMATIK URETILDI - tools/gen_px4_params.py (PX4 src yaml tarama).",
        " * Kaynak: %s (%d param). QGC MAV_AUTOPILOT_PX4 icin tam PX4" % (PX4_SRC, N),
        " * parametre seti. Degerler PX4 varsayilanlari; FC davranisina baglanma",
        " * Faz 3'te. ELLE DUZENLEME - script'i tekrar calistir. */",
        "#pragma once",
        "#include <stdint.h>",
        "",
        "#define FC_PX4_PARAM_COUNT %d" % N,
        "",
        "typedef struct {",
        "  const char *id;   /* parametre adi (<=16 char) */",
        "  uint8_t type;     /* MAV_PARAM_TYPE: 6=INT32, 9=REAL32 */",
        "  float def_val;    /* PX4 varsayilan degeri */",
        "} fc_px4_param_def_t;",
        "",
        "extern const fc_px4_param_def_t fc_px4_param_defs[FC_PX4_PARAM_COUNT];",
        "",
    ]
    with open(h, "w", encoding="utf-8") as f:
        f.write("\n".join(hdr) + "\n")

    body = [
        "/* OTOMATIK URETILDI - tools/gen_px4_params.py. Bkz fc_px4_params.h. */",
        '#include "fc_px4_params.h"',
        "",
        "const fc_px4_param_def_t fc_px4_param_defs[FC_PX4_PARAM_COUNT] = {",
    ]
    for name, (t, v) in items:
        if name in OVERRIDES:
            v = OVERRIDES[name]
        body.append('    {"%s", %d, %ff},' % (name, t, v))
    body.append("};")
    with open(c, "w", encoding="utf-8") as f:
        f.write("\n".join(body) + "\n")
    print("wrote %s + %s" % (os.path.normpath(h), os.path.normpath(c)))

    # PX4 C++ param wrapper + gercek backend icin TAM px4_parameters.hpp uret (ayni sirali items).
    hpp = os.path.join(OUT_DIR, "PX4", "src", "lib", "parameters", "px4_parameters.hpp")
    write_px4_parameters_hpp([(name, t, v) for name, (t, v) in items], hpp, "%s yaml" % PX4_SRC)


if __name__ == "__main__":
    main()
