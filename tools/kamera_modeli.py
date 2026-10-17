#!/usr/bin/env python3
"""Kamera goruntusune karanlik ve duman uygular.

Iki zorlu kosul da GORUNTU alaninda, kayit sonrasi uygulanir: tek temiz
kayittan butun seviyeler uretilir (LiDAR duman modeliyle ayni boru hatti).
LiDAR aktif bir sensor oldugu icin karanliktan etkilenmez; bu yuzden karanlik
yalnizca kamerayi, dolayisiyla RTAB-Map ve ORB-SLAM3'u bozar. Bu, raporun
"hangi algoritma hangi kosula dayanikli" sorusunun dogrudan sinanmasidir.

RADYOMETRI — neden onemli
-------------------------
Kaydedilen 8-bit goruntu gama kodludur (sRGB); piksel degeri isik siddetiyle
DOGRUSAL DEGILDIR. Hem karanlik hem duman modeli fiziksel olarak isik
siddeti uzerinde tanimli, bu yuzden once dogrusallastirilir, islem yapilir,
sonra geri kodlanir. Dogrudan piksel degeri olceklemek yanlis parlaklik
egrisi verir (orn. yarim isik, piksel degerinin yarisi degildir).

KARANLIK — neden sadece karartmak yetmez
----------------------------------------
Gercek karanlikta bozulma yalnizca goruntunun kararmasi degil, SINYAL-GURULTU
ORANININ dusmesidir: foton sayisi azalinca shot (Poisson) gurultusu baskin
hale gelir, ustune kameranin otomatik kazanci parlakligi geri getirirken
gurultuyu da yukseltir. ORB/FAST kose bulucularini oldururen budur. Model bu
zinciri izler:

    isik siddeti -> foton sayisi -> Poisson + okuma gurultusu -> kazanc -> 8 bit

Seviye parametresi AYDINLIK DUZEYI (lux). Mutlak lux degeri, simulatorun
varsayilan aydinlatmasinin iyi aydinlatilmis bir ic mekana (300 lux)
karsilik geldigi VARSAYIMINA dayali bir etikettir; bozulmanin fizigi
(SNR ~ sqrt(k)) bu capadan bagimsizdir.

DUMAN — LiDAR modeliyle ayni alpha
----------------------------------
Atmosferik sacilma modeli:

    I(x) = J(x) * t(x) + A * (1 - t(x)),     t(x) = exp(-alpha * d(x))

J temiz isik siddeti, A dumanin kendi parlakligi, d piksel derinligi
(D455'in derinlik goruntusunden GERCEK deger olarak gelir; literaturdeki zor
kisim olan tek-goruntuden derinlik kestirimi burada gereksiz). alpha AYNI
Koschmieder bagintisindan gelir, yani LiDAR ve kamera ayni "gorus mesafesi"
parametresiyle TUTARLI kademelendirilir — raporda tek bir seviye ekseni.

Derinlik goruntusu de bozulur: stereo eslestirme dusuk kontrastta basarisiz
olur, bu yuzden gecirgenligin esigin altina dustugu pikseller GECERSIZ
isaretlenir (gercek D455 dumanda derinligi boyle kaybeder).
"""
import numpy as np

KOSCHMIEDER = -np.log(0.05)      # ~2.996, LiDAR modeliyle ayni
REFERANS_LUX = 300.0             # simulator varsayilan aydinlatmasinin etiketi

# Karanlik seviyeleri (lux). Her biri oncekinin ~6-10 kati karanlik.
KARANLIK_SEVIYELERI = {
    "k0": 300.0,    # temiz: iyi aydinlatilmis ic mekan
    "k1": 50.0,     # sonuk aydinlatma / aksam ici
    "k2": 10.0,     # alacakaranlik, acil aydinlatma
    "k3": 1.0,      # dolunay
    "k4": 0.1,      # yarim ay, neredeyse karanlik
}
KARANLIK_ACIKLAMA = {
    "k0": "temiz (iyi aydinlatilmis ic mekan)",
    "k1": "sonuk aydinlatma",
    "k2": "alacakaranlik / acil aydinlatma",
    "k3": "dolunay",
    "k4": "neredeyse karanlik",
}


def sonum_katsayisi(gorus_mesafesi_m):
    if gorus_mesafesi_m <= 0:
        raise ValueError("gorus mesafesi pozitif olmali")
    return KOSCHMIEDER / gorus_mesafesi_m


# ------------------------------------------------------- sRGB <-> dogrusal
def srgb_dogrusala(u):
    """0..1 sRGB -> dogrusal isik siddeti."""
    u = np.clip(np.asarray(u, dtype=np.float32), 0.0, 1.0)
    return np.where(u <= 0.04045, u / 12.92, ((u + 0.055) / 1.055) ** 2.4)


def dogrusal_srgba(l):
    """Dogrusal isik siddeti -> 0..1 sRGB."""
    l = np.clip(np.asarray(l, dtype=np.float32), 0.0, 1.0)
    return np.where(l <= 0.0031308, l * 12.92, 1.055 * l ** (1 / 2.4) - 0.055)


# ------------------------------------------------------------------ karanlik
def karanlik_uygula(goruntu, lux, referans_lux=REFERANS_LUX,
                    kuyu_elektron=10000.0, okuma_gurultusu_e=3.0,
                    maks_kazanc=32.0, rng=None):
    """Goruntuyu verilen aydinlik duzeyine indirir.

    goruntu  : (H,W,3) ya da (H,W) uint8
    lux      : SEVIYE parametresi; kucukse karanlik
    kuyu_elektron    : doyma kapasitesi (kucuk piksel icin tipik 10k e-)
    okuma_gurultusu_e: okuma gurultusu (e-)
    maks_kazanc      : otomatik kazancin ulasabildigi tavan; bu yuzden cok
                       karanlikta goruntu hem KARANLIK hem GURULTULU kalir

    donus: (bozuk_goruntu uint8, bilgi sozlugu)
    """
    rng = np.random.default_rng() if rng is None else rng
    g8 = np.asarray(goruntu)
    k = float(lux) / float(referans_lux)        # isik siddeti olcegi

    dogrusal = srgb_dogrusala(g8.astype(np.float32) / 255.0)
    # 1) isik siddetini olcekle  2) foton/elektron sayisina cevir
    elektron = np.clip(dogrusal * k, 0.0, None) * kuyu_elektron
    # 3) shot gurultusu (Poisson) — buyuk degerlerde normal yaklasimi
    buyuk = elektron > 1e6
    gurultulu = np.where(
        buyuk,
        elektron + np.sqrt(np.maximum(elektron, 0.0)) * rng.standard_normal(elektron.shape),
        rng.poisson(np.clip(elektron, 0.0, 1e6)).astype(np.float32))
    # 4) okuma gurultusu
    gurultulu = gurultulu + okuma_gurultusu_e * rng.standard_normal(elektron.shape)
    # 5) otomatik kazanc: parlakligi geri getirmeye calisir, tavani var
    kazanc = min(1.0 / k, maks_kazanc) if k > 0 else maks_kazanc
    # 6) geri kodla ve 8 bite nicele
    cikis = dogrusal_srgba(gurultulu * kazanc / kuyu_elektron)
    cikis8 = np.clip(np.rint(cikis * 255.0), 0, 255).astype(np.uint8)

    # beklenen SNR (orta tonlu bir piksel icin), raporlanabilir olsun
    orta_e = 0.18 * k * kuyu_elektron
    snr = orta_e / np.sqrt(max(orta_e, 1e-9) + okuma_gurultusu_e ** 2)
    return cikis8, {"lux": float(lux), "olcek": k, "kazanc": kazanc,
                    "snr_ortaton": float(snr)}


# --------------------------------------------------------------------- duman
def kamera_duman_uygula(goruntu, derinlik_m, gorus_mesafesi_m,
                        atmosfer=None, maks_derinlik_m=10.0,
                        gecirgenlik_esigi=0.05, rng=None):
    """Goruntuye ve derinlik goruntusune duman uygular.

    goruntu    : (H,W,3) ya da (H,W) uint8
    derinlik_m : (H,W) float, metre. 0 / nan / inf gecersiz sayilir.
    gorus_mesafesi_m : SEVIYE parametresi (LiDAR modeliyle AYNI)
    atmosfer   : dumanin kendi dogrusal parlakligi (0..1). None ise SAHNEDEN
                 kestirilir (dogrusal siddetin %99'luk dilimi) — dehazing
                 literaturunun standardi. Sabit deger vermek yanlis: duman
                 ortam isigiyla aydinlanir, karanlik sahnede duman da
                 karanliktir; sabit 0.8 ile berrak sayilacak bir yogunlukta
                 bile goruntu beyaza boguluyordu.
    maks_derinlik_m : derinlik kirpma menzili; gecersiz pikseller bu kabul
                      edilir (uzak/gokyuzu tamamen ortulu olur)
    gecirgenlik_esigi : bunun altinda stereo eslestirme basarisiz sayilir ve
                        derinlik GECERSIZ isaretlenir

    donus: (bozuk_goruntu uint8, bozuk_derinlik float32, bilgi)
    """
    alpha = sonum_katsayisi(gorus_mesafesi_m)
    g8 = np.asarray(goruntu)
    d = np.asarray(derinlik_m, dtype=np.float32).copy()

    gecersiz = ~np.isfinite(d) | (d <= 0.0)
    d[gecersiz] = maks_derinlik_m
    d = np.clip(d, 0.0, maks_derinlik_m)

    t = np.exp(-alpha * d)                       # gecirgenlik
    t_k = t[..., None] if g8.ndim == 3 else t

    dogrusal = srgb_dogrusala(g8.astype(np.float32) / 255.0)
    if atmosfer is None:
        atmosfer = float(np.percentile(dogrusal, 99.0))
    dumanli = dogrusal * t_k + float(atmosfer) * (1.0 - t_k)
    cikis8 = np.clip(np.rint(dogrusal_srgba(dumanli) * 255.0), 0, 255).astype(np.uint8)

    # derinlik: kontrast kaybolan yerde stereo eslestirme coker
    d_cikis = np.asarray(derinlik_m, dtype=np.float32).copy()
    duman_kaybi = (t < gecirgenlik_esigi) & ~gecersiz
    kayip = duman_kaybi | gecersiz
    d_cikis[kayip] = 0.0                         # 0 = gecersiz (ROS yaklasimi)

    return cikis8, d_cikis, {
        "gorus_m": float(gorus_mesafesi_m), "alpha": float(alpha),
        "atmosfer": float(atmosfer),
        "ort_gecirgenlik": float(t[~gecersiz].mean()) if np.any(~gecersiz) else 0.0,
        # Toplam kayip, zaten gecersiz olan pikselleri (gokyuzu, menzil disi)
        # de icerir ve seviyeden bagimsiz sabit bir taban olusturur; dumanin
        # KENDI etkisi ayri raporlanir.
        "derinlik_kayip_orani": float(kayip.mean()),
        "duman_derinlik_kaybi": float(duman_kaybi.mean()),
        "zaten_gecersiz": float(gecersiz.mean()),
    }


# ------------------------------------------------------------------- metrik
def orb_sayisi(goruntu, en_fazla=2000):
    """Goruntude bulunabilen ORB kose sayisi.

    Bozulmanin SLAM icin anlamli olcusu bu: ORB-SLAM3 ve RTAB-Map'in gorsel
    odometrisi dogrudan bu kose sayisina baglidir.
    """
    import cv2
    g = np.asarray(goruntu)
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_RGB2GRAY)
    orb = cv2.ORB_create(nfeatures=en_fazla)
    return len(orb.detect(g, None))


def rms_kontrast(goruntu):
    """Goruntunun RMS kontrasti (parlaklik normalize edilmis sapma)."""
    import cv2
    g = np.asarray(goruntu)
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_RGB2GRAY)
    g = g.astype(np.float32) / 255.0
    return float(g.std())


def eslesen_kose_sayisi(g1, g2, en_fazla=2000, oran=0.75, ransac_px=3.0,
                        fast_esik=7):
    """Iki ardisik kare arasinda GUVENILIR eslesen ORB kose sayisi.

    Ham ORB sayisi yanlis metrik: karanlikta gurultu de kose gibi algilanir ve
    sayi ARTAR (olculdu: 69 -> 530), oysa goruntu kullanilamaz hale gelmistir.
    Gurultu kaynakli koseler ardisik karelerde tekrarlanmaz. Gorsel odometriyi
    (ORB-SLAM3, RTAB-Map) gercekte besleyen buyukluk, kareler arasinda eslesip
    geometrik olarak tutarli kalan kose sayisidir; bu fonksiyon onu olcer.

    Lowe oran testi + temel matris RANSAC'i ile aykirilar atilir.

    fast_esik neden 7: ORB-SLAM3 once iniThFAST=20 ile dener, yeterli kose
    bulamazsa minThFAST=7'ye duserek TEKRAR dener. Sabit 20 ile olcmek
    bozulmayi abartiyor — olculdu: 100 m gorus mesafesinde esik 20 ile
    eslesmelerin %14'u kaliyor, esik 7 ile %82'si. Siralama iki esikte de
    ayni, ama mutlak degerler algoritmanin gercek davranisiyla raporlanmali.
    """
    import cv2
    def gri(g):
        g = np.asarray(g)
        return cv2.cvtColor(g, cv2.COLOR_RGB2GRAY) if g.ndim == 3 else g
    orb = cv2.ORB_create(nfeatures=en_fazla, fastThreshold=fast_esik)
    k1, d1 = orb.detectAndCompute(gri(g1), None)
    k2, d2 = orb.detectAndCompute(gri(g2), None)
    if d1 is None or d2 is None or len(k1) < 8 or len(k2) < 8:
        return 0
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    ciftler = bf.knnMatch(d1, d2, k=2)
    iyi = [m for m, n in (c for c in ciftler if len(c) == 2)
           if m.distance < oran * n.distance]
    if len(iyi) < 8:
        return 0
    p1 = np.float32([k1[m.queryIdx].pt for m in iyi])
    p2 = np.float32([k2[m.trainIdx].pt for m in iyi])
    _, maske = cv2.findFundamentalMat(p1, p2, cv2.FM_RANSAC, ransac_px, 0.99)
    return int(maske.sum()) if maske is not None else 0
