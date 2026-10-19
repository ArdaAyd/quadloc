#!/usr/bin/env python3
"""Yapılandırma dosyasından tek bir değeri basar (kabuk betikleri için).

    tools/cfg_oku.py config/deney_v0.yaml temas.min_normal_force
"""
import sys, yaml, pathlib
d = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())
for p in sys.argv[2].split("."):
    d = d[p]
if isinstance(d, bool):
    print("true" if d else "false")
elif isinstance(d, list):
    print(" ".join(str(x) for x in d))
else:
    print(d)
