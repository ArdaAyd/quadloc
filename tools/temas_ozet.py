#!/usr/bin/env python3
"""1.2 kampanyasinin JSON ciktilarindan ozet tablo uretir.

    tools/temas_ozet.py sonuclar/12_temas
"""
import json, pathlib, sys
import numpy as np

ETIKET = {"A": "sensor, esik kapali", "B": "sensor, histerezis 15/5 N",
          "C": "yuruyus fazi"}


def ist(v):
    v = [x for x in v if x is not None and np.isfinite(x)]
    if not v:
        return "-"
    return f"{np.median(v):.4g} ({min(v):.4g}-{max(v):.4g})"


def main():
    kok = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "sonuclar/12_temas")
    kayit = {}
    hatali = []
    for f in sorted(kok.glob("12_*.json")):
        try:
            d = json.load(f.open())
        except Exception:
            hatali.append(f.stem + " (bos/bozuk json)"); continue
        if "hata" in d and "yapilandirma" not in d:
            hatali.append(f.stem); continue
        k = (d.get("yapilandirma"), d.get("hiz_ms"))
        kayit.setdefault(k, []).append(d)

    def al(ds, *yol):
        out = []
        for d in ds:
            v = d
            for p in yol:
                v = v.get(p) if isinstance(v, dict) else None
                if v is None: break
            out.append(v if isinstance(v, (int, float)) else None)
        return out

    print("# 1.2 — temas kaynagi ve bacak odometrisi")
    print("# Butun sureler ve frekanslar SIM SAATI (mesaj baslik damgalari).")
    print("# Hizalama: SE(3) Umeyama, olcek duzeltmesi yok (evo --align).")
    print("# Hucreler: medyan (min-maks)\n")
    bas = f"{'yap':<4}{'hiz':>5}{'N':>3}  {'temas orani':>20}{'adim Hz':>16}" \
          f"{'ATE rmse m':>20}{'RPE cev m':>20}{'son hata/yol':>18}"
    print(bas); print("-" * len(bas))
    for k in sorted(kayit, key=lambda x: (x[0] or "", x[1] or 0)):
        ds = kayit[k]
        print(f"{k[0]:<4}{k[1]:>5}{len(ds):>3}  "
              f"{ist(al(ds,'temas_orani_ortalama')):>20}"
              f"{ist(al(ds,'adim_frekansi_hz')):>16}"
              f"{ist(al(ds,'ate','rmse')):>20}"
              f"{ist(al(ds,'rpe_ceviri','rmse')):>20}"
              f"{ist(al(ds,'son_hata_yol_orani')):>18}")
    print("\n# Ek sutunlar (eski sayilarla karsilastirma icin): yol uzunlugu orani")
    bas2 = f"{'yap':<4}{'hiz':>5}{'N':>3}  {'yol orani (tam hiz)':>24}{'yol orani (2 Hz)':>22}{'RTF':>12}"
    print(bas2); print("-" * len(bas2))
    for k in sorted(kayit, key=lambda x: (x[0] or "", x[1] or 0)):
        ds = kayit[k]
        rtf = [float(d["rtf"]) for d in ds if d.get("rtf") not in (None, "-")]
        print(f"{k[0]:<4}{k[1]:>5}{len(ds):>3}  "
              f"{ist(al(ds,'yol_orani_tam')):>24}{ist(al(ds,'yol_orani_2hz')):>22}"
              f"{ist(rtf):>12}")
    if hatali:
        print(f"\n# BASARISIZ kosular: {', '.join(hatali)}")
    print("\n# Yapilandirmalar")
    for k, v in ETIKET.items():
        print(f"#   {k}: {v}")


if __name__ == "__main__":
    main()
