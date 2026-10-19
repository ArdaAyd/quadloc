#!/usr/bin/env python3
"""Koşunun param dump'ı ile deney yapılandırma dosyasını karşılaştırır.

    tools/param_dogrula.py <config.yaml> <param_dump_dizini>

Amaç: betiğin launch varsayılanlarına sessizce güvenmediğini KANITLAMAK.
Yapılandırmada ne yazıyorsa düğümlerde de o olmalı; fark varsa koşu geçersiz.

Çıkış: 0 fark yok, 1 fark var.
"""
import sys, pathlib, yaml

# config yolu -> (dugum adi, param adi). Dugum adi None ise hangi dugumde
# bulunursa orada aranir.
# use_sim_time'in false olmasi BEKLENEN dugumler ve gerekcesi.
SIM_ZAMANI_MUAF = {
    "gz_ros_control": "Gazebo eklentisinin kendi dugumu; simulasyon adimiyla "
                      "surulur, /clock'tan zaman almaz.",
}

ESLEME = [
    ("temas.contact_source_topic", None, None),   # topic remap, param degil
    ("temas.min_normal_force",      "foot_contacts_bridge", "min_normal_force"),
    ("temas.contact_release_force", "foot_contacts_bridge", "contact_release_force"),
    ("temas.contact_timeout",       "foot_contacts_bridge", "contact_timeout"),
    ("odometri.odom_rate",          "state_estimation_node", "odom_rate"),
]


def al(d, yol):
    for p in yol.split("."):
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


def dump_oku(dizin):
    """Her .yaml dosyasindan {dugum: {param: deger}} cikarir."""
    cikti = {}
    for f in sorted(pathlib.Path(dizin).glob("*.yaml")):
        try:
            d = yaml.safe_load(f.read_text()) or {}
        except Exception as e:
            print(f"  UYARI: {f.name} okunamadi ({e})")
            continue
        for dugum, govde in d.items():
            ad = dugum.strip("/").split("/")[-1]
            pr = (govde or {}).get("ros__parameters", {})
            cikti.setdefault(ad, {}).update(pr)
    return cikti


def main():
    if len(sys.argv) < 3:
        print(__doc__); return 2
    cfg = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())
    dump = dump_oku(sys.argv[2])
    if not dump:
        print("HATA: param dump bulunamadi"); return 1

    print(f"{'parametre':<42}{'yapilandirma':>14}{'dugumde':>14}  sonuc")
    print("-" * 84)
    fark = 0; bakilan = 0
    for yol, dugum, pad in ESLEME:
        if dugum is None:
            continue
        bekl = al(cfg, yol)
        if bekl is None:
            continue
        bulundu = None
        for d_ad, pr in dump.items():
            if d_ad == dugum and pad in pr:
                bulundu = pr[pad]; break
        bakilan += 1
        if bulundu is None:
            print(f"{yol:<42}{bekl!s:>14}{'YOK':>14}  dugumde parametre yok")
            fark += 1
        elif abs(float(bulundu) - float(bekl)) > 1e-9:
            print(f"{yol:<42}{bekl!s:>14}{bulundu!s:>14}  FARKLI")
            fark += 1
        else:
            print(f"{yol:<42}{bekl!s:>14}{bulundu!s:>14}  ayni")

    # use_sim_time: uc ayri durum. "false" ile "dump bos" ayni sey DEGIL.
    dogru, yanlis, bos = [], [], []
    for d, pr in dump.items():
        if not pr:
            bos.append(d)
        elif pr.get("use_sim_time") is True:
            dogru.append(d)
        elif d in SIM_ZAMANI_MUAF:
            pass
        else:
            yanlis.append(d)
    print("-" * 84)
    print(f"param dump'ta {len(dump)} dugum; {bakilan} parametre karsilastirildi")
    print(f"use_sim_time true : {len(dogru)} dugum")
    if yanlis:
        print(f"use_sim_time TRUE DEGIL: {', '.join(sorted(yanlis))}")
    for d, gerekce in SIM_ZAMANI_MUAF.items():
        if d in dump and dump[d].get("use_sim_time") is False:
            print(f"use_sim_time false ama MUAF: {d} — {gerekce}")
    if bos:
        print(f"param dump BOS (dugum yanit vermedi, deger bilinmiyor): "
              f"{', '.join(sorted(bos))}")
    if yanlis:
        fark += len(yanlis)
    print(f"FARK: {fark}")
    return 1 if fark else 0


if __name__ == "__main__":
    sys.exit(main())
