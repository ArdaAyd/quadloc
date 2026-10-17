#!/usr/bin/env python3
"""Kampanya gunlugunden ozet cikarir: devrilme orani, hiz, mesafe.

    tools/ozet.py [gunluk.csv]

Devrilme orani rapora dogrudan girecek bir sayi oldugu icin basarisiz denemeler
gunlukte tutuluyor; bu arac onlari sayar.
"""
import csv, os, sys
from collections import defaultdict

yol = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "kayitlar", "kampanya_gunluk.csv")
if not os.path.exists(yol):
    print(f"gunluk yok: {yol}"); sys.exit(1)

satirlar = list(csv.DictReader(open(yol, newline="")))
if not satirlar:
    print("gunluk bos"); sys.exit(1)

def sayi(s):
    try: return float(s)
    except (TypeError, ValueError): return None

toplam = len(satirlar)
sonuclar = defaultdict(int)
for s in satirlar:
    sonuclar[s["sonuc"]] += 1
gecerli = [s for s in satirlar if s["sonuc"] == "gecerli"]

print(f"gunluk         : {os.path.abspath(yol)}")
print(f"toplam deneme  : {toplam}")
for k, v in sorted(sonuclar.items(), key=lambda x: -x[1]):
    print(f"  {k:<16}: {v:4d}  ({100*v/toplam:5.1f} %)")
basarisiz = toplam - sonuclar.get("gecerli", 0)
print(f"\ndeneme basina devrilme/gecersizlik orani: {100*basarisiz/toplam:.1f} %")
print(f"gecerli kosu basina ortalama deneme      : {toplam/max(len(gecerli),1):.2f}")

benzersiz = {s["ad"] for s in gecerli}
print(f"tamamlanan benzersiz kosu                : {len(benzersiz)}")

if gecerli:
    def ort(alan):
        d = [sayi(s[alan]) for s in gecerli]
        d = [x for x in d if x is not None]
        return sum(d)/len(d) if d else float("nan")
    print(f"\ngecerli kosularda:")
    print(f"  ortalama mesafe : {ort('mesafe_m'):.2f} m")
    print(f"  ortalama hiz    : {ort('hiz_ms'):.3f} m/s (sim zamani)")
    print(f"  ortalama RTF    : {ort('rtf'):.3f}")
    print(f"  ortalama maks roll: {ort('maks_roll_rad'):.2f} rad")

    ortam = defaultdict(lambda: [0, 0])
    for s in satirlar:
        ortam[s["dunya"]][0] += 1
        if s["sonuc"] == "gecerli":
            ortam[s["dunya"]][1] += 1
    if len(ortam) > 1:
        print("\nortam basina:")
        for d, (t, g) in sorted(ortam.items()):
            print(f"  {d:<12}: {g}/{t} gecerli ({100*(t-g)/t:.0f} % gecersiz)")
