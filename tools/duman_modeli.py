#!/usr/bin/env python3
"""LiDAR nokta bulutuna duman/sis uygular (Hahner vd., ICCV 2021 yonteminin
tek-sacilma yaklasimi).

Neden simulatorde degil burada: Gazebo'nun parcacik sistemi LiDAR'i etkiliyor
ama KADEMELENDIRILEMIYOR — yogunluk 60 kat artirilsa da ortme etkisi ayni
kaliyor (bkz. DURUM.md). Burada seviye, fiziksel bir buyukluk olan GORUS
MESAFESI ile veriliyor ve surekli degisken.

Fizik
-----
Sonum katsayisi, Koschmieder bagintisiyla gorus mesafesinden gelir:

    alpha = -ln(0.05) / V  ~=  2.996 / V        [1/m]

Isinin gidis-donus yolu 2R oldugu icin sert hedeften donen guc

    P_sert = i * exp(-2 * alpha * R)

kadar zayiflar. Ayrica isin, hedefe varmadan once duman parcaciklarina
carpabilir: tek-sacilma yaklasiminda ilk sacilmanin r mesafesinde olma
olasilik yogunlugu alpha*exp(-alpha*r)'dir. Sacilma hedeften once olursa
(r < R) ve geri donen guc sert hedefinkini asarsa, sensor NOKTAYI DAHA YAKINDA
GORUR. Zayiflamis sert donus algilama esiginin altina duserse nokta tamamen
KAYBOLUR.

Yani model gercekte gozlenen iki etkiyi de uretir: sahte yakin donusler ve
uzaktaki yapinin kaybolmasi.

Sinirlar
--------
- Tek sacilma varsayimi; cok yogun dumanda (V < ~5 m) coklu sacilma ihmal edilir.
- Yogunluk uniform kabul edilir (duman bulutunun uzamsal yapisi yok).
- Kamera goruntusune uygulanmaz; gorus alani modeli ayrica gerekir.
"""
import numpy as np

# Koschmieder: %5 kontrast esigi
KOSCHMIEDER = -np.log(0.05)      # ~2.996


def sonum_katsayisi(gorus_mesafesi_m):
    """Gorus mesafesinden (m) sonum katsayisi (1/m)."""
    if gorus_mesafesi_m <= 0:
        raise ValueError("gorus mesafesi pozitif olmali")
    return KOSCHMIEDER / gorus_mesafesi_m


def duman_uygula(noktalar, gorus_mesafesi_m, yogunluk=None,
                 maks_menzil_m=30.0, referans_yansima=0.1,
                 en_yakin_m=0.5, tohum=0):
    """Nokta bulutuna duman uygular.

    noktalar : (N,3) veya (N,4) dizi — x, y, z [, yogunluk]
    gorus_mesafesi_m : SEVIYE parametresi; kucukse yogun duman
    maks_menzil_m : LiDAR'in berrak havadaki anma menzili (VLP-16 icin 30 m)
    referans_yansima : anma menzilinin tanimlandigi hedef yansimasi (tipik 0.1)
    en_yakin_m : olculebilen en kisa mesafe

    Butun gucler ayni olcekte (1/r^2 dahil) hesaplanir; algilama esigi de
    bu olcekten turetilir: berrak havada anma menzilindeki referans hedefin
    dondurdugu guc. Ilk surumde esik 1/r^2 icermeyen bir buyuklukle
    karsilastiriliyordu ve sonuclar tutarsizdi.

    Doner: (yeni_noktalar, etiketler)
      etiketler: 0 = degismedi, 1 = duman donusu, 2 = silindi
    """
    p = np.asarray(noktalar, dtype=float)
    xyz = p[:, :3]
    if yogunluk is None:
        yogunluk = p[:, 3] if p.shape[1] > 3 else np.ones(len(p))
    yogunluk = np.asarray(yogunluk, dtype=float)

    # Gercek LiDAR bulutlarinda, hicbir seye carpmayan isinlar sonsuz (veya cok
    # buyuk) koordinatlarla geliyor. Bunlar bilgi tasimadigi gibi modelde
    # inf/nan uretiyordu; basta ayiklaniyorlar.
    sonlu = np.all(np.isfinite(xyz), axis=1)
    if not np.all(sonlu):
        xyz = xyz[sonlu]
        yogunluk = yogunluk[sonlu]

    rng = np.random.default_rng(tohum)
    alpha = sonum_katsayisi(gorus_mesafesi_m)
    beta = alpha / (4.0 * np.pi)          # geri sacilma, ayni duman turu icin alpha ile orantili

    R = np.maximum(np.linalg.norm(xyz, axis=1), en_yakin_m)
    etiket = np.zeros(len(xyz), dtype=int)

    # algilama esigi: berrak havada anma menzilindeki referans hedefin gucu
    P_min = referans_yansima / maks_menzil_m**2

    # sert hedef: gidis-donus sonumu + 1/R^2
    P_sert = yogunluk * np.exp(-2.0 * alpha * R) / R**2

    # ilk sacilma mesafesi: alpha*exp(-alpha*r) dagilimi (ters donusum)
    u = rng.random(len(xyz))
    r_sac = -np.log(np.clip(u, 1e-12, 1.0)) / alpha
    sacilma_var = (r_sac > en_yakin_m) & (r_sac < R)
    P_duman = np.where(sacilma_var, beta * np.exp(-2.0 * alpha * r_sac) / r_sac**2, 0.0)

    yeni = xyz.copy()
    yeni_yogunluk = P_sert.copy()

    # duman donusu hem sert donusu bastirmali hem algilanabilir olmali
    duman_kazanir = sacilma_var & (P_duman > P_sert) & (P_duman > P_min)
    if np.any(duman_kazanir):
        yeni[duman_kazanir] = xyz[duman_kazanir] * (r_sac[duman_kazanir] / R[duman_kazanir])[:, None]
        yeni_yogunluk[duman_kazanir] = P_duman[duman_kazanir]
        etiket[duman_kazanir] = 1

    # duman kurtarmadiysa ve zayiflamis donus esigin altindaysa nokta kaybolur
    kayip = (~duman_kazanir) & (P_sert < P_min)
    etiket[kayip] = 2

    kalan = etiket != 2
    return np.column_stack([yeni[kalan], yeni_yogunluk[kalan]]), etiket


if __name__ == "__main__":
    # Hizli dogrulama: 1..30 m arasi duz bir isin demeti
    R = np.linspace(1, 30, 3000)
    noktalar = np.column_stack([R, np.zeros_like(R), np.zeros_like(R)])
    print("gorus_m;kalan_nokta_%;duman_donusu_%;ort_menzil_m;30m'ye_ulasan_%")
    for V in (1000, 100, 50, 20, 10, 5):
        yeni, et = duman_uygula(noktalar, V, tohum=1)
        kalan = 100 * len(yeni) / len(noktalar)
        duman = 100 * np.mean(et == 1)
        r = np.linalg.norm(yeni[:, :3], axis=1)
        uzak = 100 * np.mean(r > 25)
        print(f"{V};{kalan:.1f};{duman:.1f};{r.mean():.2f};{uzak:.1f}")
