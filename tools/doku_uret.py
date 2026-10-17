#!/usr/bin/env python3
"""Yordamsal (procedural) doku goruntuleri uretir.

    tools/doku_uret.py [--cikti DIZIN] [--boyut 1024] [--duvar-sayisi 8]

Neden gerekli: mevcut dunyalarin hepsi duz gri kutulardan olusuyor. Temiz,
iyi aydinlatilmis bir goruntude bile yalnizca 57 ORB kosesi ve ardisik kareler
arasinda 27 eslesme olculdu (bkz. DURUM.md). ORB-SLAM3'un baslatma icin
yuzlerce kararli koseye ihtiyaci var; RTAB-Map'in gorsel odometrisi de ayni
sekilde. Dokusuz dunyada gorsel SLAM HER kosulda — temiz dahil — basarisiz
olur, dolayisiyla kosullar arasinda fark olculemez.

Hazir varlik indirmek yerine doku uretilmesinin iki sebebi var: internet
baglantisi ve Fuel onbellegi gerektirmiyor, ve doku zenginligi KONTROLLU bir
degisken haline geliyor (gerekirse kasitli olarak zayif dokulu bir ortam da
uretilebilir).

Dokular ORB'un ihtiyacini karsilayacak sekilde tasarlandi:
  - cok olcekli ayrinti: genis paneller + ince benek (FAST farkli olceklerde
    kose bulur)
  - yuksek kontrastli kose kaynaklari: levha/afis dikdortgenleri
  - tekrarsizlik: her panel farkli tohumla uretilir, boylece paneller
    arasinda yanlis eslesme olmaz (tekrarli doku eslestirmeyi belirsizlestirir)
"""
import argparse, os
import numpy as np


def _coklu_oktav(sekil, rng, oktav=5, sonum=0.55):
    """Cok olcekli deger gurultusu (0..1)."""
    h, w = sekil
    toplam = np.zeros(sekil, np.float32)
    genlik = 1.0
    bolen = 0.0
    for k in range(oktav):
        n = 2 ** (k + 2)
        kaba = rng.random((min(n, h), min(n, w))).astype(np.float32)
        # bilinear buyutme
        yi = np.linspace(0, kaba.shape[0] - 1, h)
        xi = np.linspace(0, kaba.shape[1] - 1, w)
        y0 = np.floor(yi).astype(int); x0 = np.floor(xi).astype(int)
        y1 = np.minimum(y0 + 1, kaba.shape[0] - 1)
        x1 = np.minimum(x0 + 1, kaba.shape[1] - 1)
        fy = (yi - y0)[:, None]; fx = (xi - x0)[None, :]
        ust = kaba[np.ix_(y0, x0)] * (1 - fx) + kaba[np.ix_(y0, x1)] * fx
        alt = kaba[np.ix_(y1, x0)] * (1 - fx) + kaba[np.ix_(y1, x1)] * fx
        toplam += genlik * (ust * (1 - fy) + alt * fy)
        bolen += genlik
        genlik *= sonum
    return toplam / bolen


def _afisler(img, rng, sayi=12, en_kucuk=0.05, en_buyuk=0.30):
    """Yuksek kontrastli dikdortgen levhalar: kose kaynagi.

    Boyutlar PANEL ORANI olarak verilir (0.05 = panelin %5'i). Render'da 1 m'lik
    panel yalnizca ~100-140 piksele dustugu icin, ayrintinin panelin en az
    %5'i kadar buyuk olmasi gerekir; daha incesi yeniden orneklemede ortalanip
    kayboluyor (olculdu: ince benekli ilk surum 100 px'te 33 kose veriyordu).
    """
    h, w = img.shape[:2]
    for _ in range(sayi):
        ph = int(rng.integers(int(h*en_kucuk), int(h*en_buyuk)))
        pw = int(rng.integers(int(w*en_kucuk), int(w*en_buyuk)))
        y = int(rng.integers(0, h - ph)); x = int(rng.integers(0, w - pw))
        taban = rng.random(3).astype(np.float32)
        taban = taban * 0.85 + (0.05 if taban.mean() < 0.5 else 0.10)
        img[y:y + ph, x:x + pw] = taban
        karsit = 1.0 - taban
        # ic desen: kalin cubuklar/bloklar (ince cizgi render'da kayboluyor)
        for _ in range(int(rng.integers(2, 5))):
            kal = max(4, int(min(ph, pw) * rng.uniform(0.12, 0.3)))
            if rng.random() < 0.5:
                sy = int(rng.integers(y, max(y + 1, y + ph - kal)))
                img[sy:sy + kal, x:x + pw] = karsit
            else:
                sx = int(rng.integers(x, max(x + 1, x + pw - kal)))
                img[y:y + ph, sx:sx + kal] = karsit
        cer = max(3, int(min(ph, pw) * 0.08))          # kalin cerceve
        img[y:y + cer, x:x + pw] = 0.04; img[y + ph - cer:y + ph, x:x + pw] = 0.04
        img[y:y + ph, x:x + cer] = 0.04; img[y:y + ph, x + pw - cer:x + pw] = 0.04
    return img


def duvar_dokusu(boyut, tohum, tugla_y=8, tugla_x=4):
    """Tugla/panel desenli duvar dokusu.

    Butun ayrintilar RENDER OLCEGINDE hayatta kalacak sekilde boyutlandirildi:
    harc ~2 cm, tugla 25 x 12 cm, levhalar 5-30 cm. Ince benek yalnizca yakin
    plan gercekciligi icin, kucuk genlikle eklenir.
    """
    rng = np.random.default_rng(tohum)
    n = boyut
    # dusuk frekansli leke (az oktav = buyuk yapi)
    taban = 0.40 + 0.30 * _coklu_oktav((n, n), rng, oktav=3)
    img = np.repeat(taban[:, :, None], 3, axis=2)
    img *= (0.88 + 0.24 * rng.random(3)).astype(np.float32)

    sy, sx = n // tugla_y, n // tugla_x
    harc = max(6, n // 50)                      # ~2 cm: yeniden orneklemede kalir
    for i in range(tugla_y):
        kaydir = (sx // 2) if i % 2 else 0
        for j in range(-1, tugla_x + 1):
            y0, x0 = i * sy, j * sx + kaydir
            y1, x1 = min(y0 + sy, n), min(x0 + sx, n)
            if x1 <= 0 or x0 >= n:
                continue
            img[y0:y1, max(x0, 0):x1] *= 0.60 + 0.75 * rng.random()   # guclu ton farki
        img[i * sy:i * sy + harc, :] *= 0.30                          # yatay harc
    for j in range(tugla_x + 1):
        for i in range(tugla_y):
            x = j * sx + ((sx // 2) if i % 2 else 0)
            if 0 <= x < n:
                img[i * sy:(i + 1) * sy, x:x + harc] *= 0.30          # dusey harc

    # buyuk koyu lekeler (orta olcek kose kaynagi)
    for _ in range(int(rng.integers(4, 9))):
        rr = int(rng.integers(n // 12, n // 5))
        yy, xx = int(rng.integers(0, n)), int(rng.integers(0, n))
        y0, y1 = max(0, yy - rr), min(n, yy + rr)
        x0, x1 = max(0, xx - rr), min(n, xx + rr)
        img[y0:y1, x0:x1] *= 0.55 + 0.3 * rng.random()

    img = _afisler(img, rng, sayi=int(rng.integers(10, 16)))
    # ince benek: yalnizca yakin planda gorunur, kucuk genlik
    img *= (0.94 + 0.12 * rng.random((n, n)).astype(np.float32))[:, :, None]
    return np.clip(img, 0, 1)


def zemin_dokusu(boyut, tohum, kare=4):
    """Fayans izli beton zemin.

    Zemin karesi 2 m uzunlugunda, yani 1024 px = 2 m. Karo 50 cm, derz ~3 cm.
    Zeminin kadraji kaplayan payi buyuk oldugu icin kontrastinin yeterli olmasi
    onemli; ilk surumde zemin 142 px'te yalnizca 16 kose veriyordu.
    """
    rng = np.random.default_rng(tohum)
    n = boyut
    taban = 0.35 + 0.30 * _coklu_oktav((n, n), rng, oktav=3)
    img = np.repeat(taban[:, :, None], 3, axis=2)
    s_ = n // kare
    derz = max(6, n // 60)
    for i in range(kare):
        for j in range(kare):
            img[i*s_:(i+1)*s_, j*s_:(j+1)*s_] *= 0.70 + 0.60 * rng.random()
    for i in range(kare + 1):
        img[min(i*s_, n-derz):min(i*s_+derz, n), :] *= 0.35
        img[:, min(i*s_, n-derz):min(i*s_+derz, n)] *= 0.35
    # izler/lekeler: orta olcek
    for _ in range(int(rng.integers(14, 24))):
        rr = int(rng.integers(n // 40, n // 10))
        yy, xx = int(rng.integers(0, n)), int(rng.integers(0, n))
        y0, y1 = max(0, yy - rr), min(n, yy + rr)
        x0, x1 = max(0, xx - rr), min(n, xx + rr)
        img[y0:y1, x0:x1] *= 0.45 + 0.45 * rng.random()
    img *= (0.94 + 0.12 * rng.random((n, n)).astype(np.float32))[:, :, None]
    return np.clip(img, 0, 1)


def main():
    import cv2
    ap = argparse.ArgumentParser()
    ap.add_argument("--cikti", default=None,
                    help="varsayilan: unitree_go2_description/materials/textures")
    ap.add_argument("--boyut", type=int, default=1024)
    ap.add_argument("--duvar-sayisi", type=int, default=8)
    ap.add_argument("--tohum", type=int, default=0)
    a = ap.parse_args()

    kok = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cikti = a.cikti or os.path.join(
        kok, "ws/src/unitree_go2_ros2/unitree_go2_description/materials/textures")
    os.makedirs(cikti, exist_ok=True)

    for i in range(a.duvar_sayisi):
        img = duvar_dokusu(a.boyut, a.tohum * 1000 + i)
        yol = os.path.join(cikti, f"duvar_{i}.png")
        cv2.imwrite(yol, cv2.cvtColor((img * 255).astype(np.uint8),
                                      cv2.COLOR_RGB2BGR))
        print("yazildi:", yol)
    img = zemin_dokusu(a.boyut, a.tohum * 1000 + 777)
    yol = os.path.join(cikti, "zemin.png")
    cv2.imwrite(yol, cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
    print("yazildi:", yol)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
