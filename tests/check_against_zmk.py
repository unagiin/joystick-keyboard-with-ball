"""Static checks of our config against a ZMK checkout, without compiling anything.

Usage: check_against_zmk.py <west workspace> <firmware folder>
This reads text only and is no substitute for a real build.
"""

import re
import sys
from pathlib import Path

ws, fw = Path(sys.argv[1]), Path(sys.argv[2])
problems = []

keymap = (fw / "config/joyball42.keymap").read_text(encoding="utf-8")
keys_h = (ws / "zmk/app/include/dt-bindings/zmk/keys.h").read_text(encoding="utf-8")
defined = set(re.findall(r"#define\s+(\w+)", keys_h))

used = set(re.findall(r"&kp\s+(\w+)", keymap)) | set(re.findall(r"&lt\s+\w+\s+(\w+)", keymap))
missing = sorted(used - defined)
print(f"key codes used: {len(used)}, undefined: {missing}")
problems += [f"undefined key {k}" for k in missing]

pointing = (ws / "zmk/app/include/dt-bindings/zmk/pointing.h").read_text(encoding="utf-8")
for button in set(re.findall(r"&mkp\s+(\w+)", keymap)):
    if not re.search(rf"#define\s+{button}\b", pointing):
        problems.append(f"undefined mouse button {button}")

dts = "\n".join(p.read_text(encoding="utf-8", errors="replace")
                for p in (ws / "zmk/app/dts").rglob("*.dtsi"))
for label in ("zip_temp_layer", "zip_xy_scaler", "zip_xy_to_scroll_mapper", "bootloader", "sys_reset"):
    found = re.search(rf"\b{label}\s*:", dts) is not None
    print(f"label {label}: {found}")
    if not found:
        problems.append(f"label {label} not in ZMK dts")

binding_dirs = [ws / "zmk/app/dts/bindings", ws / "zmk/app/module/dts/bindings",
                ws / "zmk-pmw3610-driver/dts/bindings", fw / "dts/bindings"]
bindings = {p.stem for d in binding_dirs if d.exists() for p in d.rglob("*.y*ml")}
overlays = [p.read_text(encoding="utf-8") for p in (fw / "boards/shields/joyball42").glob("*")
            if p.suffix in (".overlay", ".dtsi")]
for compat in sorted({c for text in overlays for c in re.findall(r'compatible = "([^"]+)"', text)}):
    if compat.startswith("nordic,"):
        continue
    ok = compat in bindings
    print(f"binding {compat}: {ok}")
    if not ok:
        problems.append(f"no binding for {compat}")

kconfig = "\n".join(p.read_text(encoding="utf-8", errors="replace")
                    for root in (ws / "zmk/app", ws / "zmk-pmw3610-driver", fw)
                    for p in root.rglob("Kconfig*"))
kconfig += (ws / "zephyr/Kconfig.zephyr").read_text(encoding="utf-8", errors="replace")
kconfig += (ws / "zephyr/soc/arm/nordic_nrf/Kconfig").read_text(encoding="utf-8", errors="replace")
confs = list((fw / "config").glob("*.conf")) + list((fw / "boards/shields/joyball42").glob("*.conf"))
for conf in confs:
    for symbol in re.findall(r"^CONFIG_(\w+)=", conf.read_text(encoding="utf-8"), re.M):
        if not re.search(rf"^\s*(menu)?config\s+{symbol}\b", kconfig, re.M) and not symbol.startswith(("BT_", "SPI", "ADC", "INPUT")):
            problems.append(f"{conf.name}: CONFIG_{symbol} not found")
            print(f"Kconfig {symbol}: NOT FOUND")

shield = fw / "boards/shields/joyball42"
dtsi = (shield / "joyball42.dtsi").read_text(encoding="utf-8")
positions = len(re.findall(r"RC\(\d+,\d+\)", dtsi))
physical = (shield / "joyball42_layout.dtsi").read_text(encoding="utf-8").count("&key_physical_attrs")
print(f"transform positions: {positions}, physical layout keys: {physical}")
if positions != physical:
    problems.append("physical layout and matrix transform differ in length")
binding_tokens = re.compile(r"&\w+")
for name, body in re.findall(r'display-name = "(\w+)";\s*bindings = <(.*?)>;', keymap, re.S):
    count = len(binding_tokens.findall(body))
    print(f"layer {name}: {count}")
    if count != positions:
        problems.append(f"layer {name} has {count} bindings, expected {positions}")

print("\n".join(problems) if problems else "no problems found by this static check")
sys.exit(1 if problems else 0)
