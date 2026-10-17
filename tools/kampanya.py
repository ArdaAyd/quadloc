#!/usr/bin/env python3
"""Deney matrisini kosar, kaldigi yerden devam eder, ozet cikarir.

    tools/kampanya.py --dunyalar warehouse,default --tekrar 3 --sure 60
    tools/kampanya.py --kuru            # sadece ne kosulacagini listeler

Onemli planlama notu: SLAM algoritmalari kayitli bag uzerinde calisacagi icin
her (ortam x kosul x seviye x tekrar) kombinasyonu icin SIMULASYON yalnizca
BIR kez kosulur; ayni bag uc algoritmaya da beslenir. Yani 3 ortam x 4 kosul x
4 seviye x 3 tekrar = 144 simulasyon kosusu, 432 degil. 432 olan, bag'ler
uzerinde yapilacak algoritma degerlendirmesi.

Kaldigi yerden devam: kayitlar/kampanya_gunluk.csv'de 'gecerli' satiri olan ve
kaydi diskte duran kombinasyonlar atlanir. Kampanyayi istediginiz zaman
kesebilir, sonra ayni komutla devam edebilirsiniz.
"""
import argparse, csv, os, subprocess, sys, time

KOK = os.path.dirname(os.path.abspath(__file__))
KAYIT = os.path.join(KOK, "..", "kayitlar")
GUNLUK = os.path.join(KAYIT, "kampanya_gunluk.csv")


def gecerli_kosular():
    """Gunlukte 'gecerli' yazan ve kaydi hala diskte duran kosu adlari."""
    if not os.path.exists(GUNLUK):
        return set()
    bitmis = set()
    with open(GUNLUK, newline="") as f:
        for satir in csv.DictReader(f):
            if satir.get("sonuc") == "gecerli":
                ad = satir["ad"]
                if os.path.isdir(os.path.join(KAYIT, ad)):
                    bitmis.add(ad)
    return bitmis


def ad_uret(dunya, kosul, seviye, tekrar):
    return f"{dunya}__{kosul}_s{seviye}__r{tekrar}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dunyalar", default="warehouse",
                    help="virgulle ayrilmis (warehouse,default,baylands)")
    ap.add_argument("--kosullar", default="temiz",
                    help="zorlu kosul turleri; IP2 ile duman,toz,karanlik,imu,kayganlik eklenecek")
    ap.add_argument("--seviyeler", default="0",
                    help="kosul siddet seviyeleri, virgulle (0,1,2,3)")
    ap.add_argument("--tekrar", type=int, default=3)
    ap.add_argument("--sure", type=float, default=60.0, help="simulasyon saniyesi")
    ap.add_argument("--ly", type=float, default=1.0)
    ap.add_argument("--profil", default="lidar", choices=["lidar", "kamera", "tumu"])
    ap.add_argument("--en-fazla-deneme", type=int, default=4)
    ap.add_argument("--kuru", action="store_true", help="kosma, sadece listele")
    a = ap.parse_args()

    dunyalar = [x.strip() for x in a.dunyalar.split(",") if x.strip()]
    kosullar = [x.strip() for x in a.kosullar.split(",") if x.strip()]
    seviyeler = [x.strip() for x in a.seviyeler.split(",") if x.strip()]

    matris = [(d, k, s, r)
              for d in dunyalar for k in kosullar
              for s in seviyeler for r in range(1, a.tekrar + 1)]
    bitmis = gecerli_kosular()
    bekleyen = [m for m in matris if ad_uret(*m) not in bitmis]

    print(f"matris       : {len(matris)} kombinasyon "
          f"({len(dunyalar)} ortam x {len(kosullar)} kosul x "
          f"{len(seviyeler)} seviye x {a.tekrar} tekrar)")
    print(f"zaten bitmis : {len(matris) - len(bekleyen)}")
    print(f"kosulacak    : {len(bekleyen)}")
    if a.kuru:
        for m in bekleyen:
            print("   ", ad_uret(*m))
        return 0
    if not bekleyen:
        print("yapilacak is yok.")
        return 0

    basari = basarisiz = 0
    sureler = []
    t_kampanya = time.time()
    for i, (dunya, kosul, seviye, tekrar) in enumerate(bekleyen, 1):
        ad = ad_uret(dunya, kosul, seviye, tekrar)
        kalan = len(bekleyen) - i + 1
        tahmin = (f"  ~{kalan * (sum(sureler)/len(sureler)) / 60:.0f} dk kaldi"
                  if sureler else "")
        print(f"\n########## [{i}/{len(bekleyen)}] {ad}{tahmin}", flush=True)
        t0 = time.time()
        kod = subprocess.call([os.path.join(KOK, "deney.sh"), ad,
                               str(a.sure), str(a.ly), dunya, a.profil,
                               str(a.en_fazla_deneme)])
        sureler.append(time.time() - t0)
        if kod == 0:
            basari += 1
        else:
            basarisiz += 1
            print(f"!!! {ad} tamamlanamadi ({a.en_fazla_deneme} deneme)", flush=True)

    print(f"\n==================== KAMPANYA OZETI ====================")
    print(f"gecerli kosu   : {basari}")
    print(f"tamamlanamayan : {basarisiz}")
    print(f"toplam sure    : {(time.time()-t_kampanya)/60:.1f} dk")
    print(f"gunluk         : {GUNLUK}")
    return 0 if basarisiz == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
