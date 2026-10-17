#!/usr/bin/env python3
"""IMU olcumlerine gercekci MEMS hata modeli uygular.

Neden post-isleme: Gazebo'nun imu sensorune gurultu verilebilir ama seviye
degistirmek icin her seferinde yeniden kosmak gerekir. Burada tek temiz
kayittan istenildigi kadar seviye uretilir; duman modeliyle ayni boru hatti.

Hata modeli (Allan varyans parametrelerine dayali, standart INS literaturu)
---------------------------------------------------------------------------
Jiroskop:
    w_olcum = (1 + s_g) * w + b_g(t) + n_g(t)
Ivmeolcer:
    a_olcum = (1 + s_a) * a + b_a(t) + n_a(t)

  n : beyaz gurultu. Yogunluk N (rad/s/sqrt(Hz)) verildiginde f Hz'de
      orneklenen ayrik sapma  sigma = N * sqrt(f).
  b : surus-ici bias. Birinci dereceden Gauss-Markov sureci olarak
      modellenir (navigasyon literaturunun standardi):
          db = -(b/tau) dt + sigma_b * sqrt(2 dt / tau) * N(0,1)
      kararli durum sapmasi sigma_b = datasheet'teki "in-run bias stability".
      Buna ek olarak acilista kalibrasyondan ARTAN sabit bir bias eklenir
      (gercek suruculer durarak bias kestirir, ama tamamen gideremez).
  s : olcek katsayisi hatasi, kosu boyunca sabit, eksen basina.

Yonelim (orientation) alani
---------------------------
IMU mesaji ham gyro/ivme disinda bir de yonelim tasir ve bunu BAZI tuketiciler
kullanir (RTAB-Map yercekimi kisiti, bizim bacak odometrisinin roll/pitch
duzeltmesi, EKF). Sadece ham veriyi bozup yonelimi temiz birakmak testi
sahte yapardi; yonelim de AYNI hata gerceklemesinden tutarli sekilde bozulur:

  - roll/pitch: ivmeolcerle referanslanir, yani SINIRLI hata.
        d_roll  =  b_ay / g      d_pitch = -b_ax / g
  - yaw: mutlak referansi yoktur, gyro-z entegrasyonuyla SURUKLENIR.
        d_yaw(t) = integral( s_gz * w_z + b_gz + n_gz ) dt

Bu fark (differential) yaklasimi secildi; alternatif, cihazin kendi
tamamlayici filtresini bastan simule etmekti. O yol reddedildi cunku
tirisla kosan bir robotta filtrenin KENDI takip hatasi (ivme sapmasi 6 m/s^2)
olcume karisir ve gurultu sifirken bile hata uretir. Bu yaklasimda gurultu
sifirsa yonelim hatasi tam olarak sifirdir (dogrulandi).
"""
import numpy as np

G = 9.80665
DER = np.pi / 180.0          # derece -> radyan
MIKRO_G = 1e-6 * G           # ug -> m/s^2
MILI_G = 1e-3 * G            # mg  -> m/s^2

# Seviyeler: her biri oncekinin ~3 kati. Fiziksel birimlerde verilir ki
# raporda "gorus mesafesi" gibi olculebilir bir buyuklukle kademelendirilsin.
#   gyro_yogunluk : deg/s/sqrt(Hz)      ivme_yogunluk : ug/sqrt(Hz)
#   gyro_bias     : deg/h  (surus-ici)  ivme_bias     : mg (surus-ici)
#   olcek         : %  (sapma)
SEVIYELER = {
    "s0": dict(gyro_yogunluk=0.0,  gyro_bias=0.0,    ivme_yogunluk=0.0,
               ivme_bias=0.0,   olcek=0.0),
    "s1": dict(gyro_yogunluk=0.01, gyro_bias=20.0,   ivme_yogunluk=100.0,
               ivme_bias=2.0,   olcek=0.1),
    "s2": dict(gyro_yogunluk=0.03, gyro_bias=100.0,  ivme_yogunluk=300.0,
               ivme_bias=10.0,  olcek=0.5),
    "s3": dict(gyro_yogunluk=0.10, gyro_bias=500.0,  ivme_yogunluk=1000.0,
               ivme_bias=40.0,  olcek=1.5),
    "s4": dict(gyro_yogunluk=0.30, gyro_bias=2000.0, ivme_yogunluk=3000.0,
               ivme_bias=150.0, olcek=4.0),
}

SEVIYE_ACIKLAMA = {
    "s0": "temiz (simulator varsayilani)",
    "s1": "iyi endustriyel sinif",
    "s2": "tipik tuketici MEMS",
    "s3": "ucuz MEMS / titresim altinda",
    "s4": "arizali sensor",
}


def _gauss_markov(n, sapma, tau, dt, rng):
    """Birinci dereceden Gauss-Markov bias izi (3 eksen)."""
    b = np.zeros((n, 3))
    if sapma <= 0 or n == 0:
        return b
    # kararli durumdan basla, boylece ilk ornekler de temsili
    b[0] = rng.normal(0.0, sapma, 3)
    katsayi = np.exp(-dt / tau)
    itki = sapma * np.sqrt(1.0 - katsayi ** 2)
    for i in range(1, n):
        b[i] = katsayi * b[i - 1] + itki * rng.normal(0.0, 1.0, 3)
    return b


class ImuBozucu:
    """Bir kosu boyunca tutarli tek bir hata gerceklemesi uretir.

    Olcek hatasi ve acilis biasi kosu boyunca SABIT; beyaz gurultu ve
    surus-ici bias zamana bagli. Ayni tohum ayni izi verir.
    """

    def __init__(self, seviye=None, hiz_hz=100.0, tau_s=100.0,
                 acilis_carpan=1.0, tohum=0, **ustyaz):
        p = dict(SEVIYELER["s0"])
        if seviye is not None:
            if seviye not in SEVIYELER:
                raise ValueError(f"bilinmeyen seviye {seviye!r}; "
                                 f"secenekler {sorted(SEVIYELER)}")
            p.update(SEVIYELER[seviye])
        p.update({k: v for k, v in ustyaz.items() if v is not None})
        self.seviye = seviye
        self.p = p
        self.hiz_hz = float(hiz_hz)
        self.tau_s = float(tau_s)
        self.acilis_carpan = float(acilis_carpan)
        self.rng = np.random.default_rng(tohum)

        # --- SI birimlerine cevir ---
        # beyaz gurultu: ayrik sapma = yogunluk * sqrt(f)
        self.sigma_g = p["gyro_yogunluk"] * DER * np.sqrt(self.hiz_hz)
        self.sigma_a = p["ivme_yogunluk"] * MIKRO_G * np.sqrt(self.hiz_hz)
        # surus-ici bias kararli durum sapmasi
        self.bias_g = p["gyro_bias"] * DER / 3600.0        # deg/h -> rad/s
        self.bias_a = p["ivme_bias"] * MILI_G              # mg -> m/s^2
        # acilista kalibrasyondan artan sabit bias
        self.acilis_g = self.rng.normal(0.0, self.bias_g * self.acilis_carpan, 3)
        self.acilis_a = self.rng.normal(0.0, self.bias_a * self.acilis_carpan, 3)
        # olcek katsayisi hatasi (eksen basina sabit)
        self.olcek_g = self.rng.normal(0.0, p["olcek"] / 100.0, 3)
        self.olcek_a = self.rng.normal(0.0, p["olcek"] / 100.0, 3)

    # ---------------------------------------------------------------- bilgi
    def ozet(self):
        p = self.p
        return (f"gyro {p['gyro_yogunluk']:.3g} deg/s/sqrt(Hz) "
                f"(sigma {self.sigma_g:.2e} rad/s), bias {p['gyro_bias']:.0f} deg/h "
                f"({self.bias_g:.2e} rad/s) | "
                f"ivme {p['ivme_yogunluk']:.0f} ug/sqrt(Hz) "
                f"(sigma {self.sigma_a:.2e} m/s^2), bias {p['ivme_bias']:.3g} mg | "
                f"olcek %{p['olcek']:.2g}")

    def beklenen_yaw_suruklenmesi(self, sure_s):
        """Bias kaynakli yaw suruklenmesinin kaba buyuklugu (derece).

        Sabit acilis biasi b0*t, Gauss-Markov kismi ise tau << t icin
        yaklasik sigma*sqrt(2*tau*t) kadar katki verir.
        """
        sabit = abs(self.acilis_g[2]) * sure_s
        gezinen = self.bias_g * np.sqrt(2.0 * self.tau_s * sure_s)
        beyaz = self.p["gyro_yogunluk"] * DER * np.sqrt(sure_s)
        return np.hypot(np.hypot(sabit, gezinen), beyaz) / DER

    # -------------------------------------------------------------- uygulama
    def iz_uret(self, n, dt=None):
        """n ornek icin zaman degisken hata izlerini uretir."""
        dt = (1.0 / self.hiz_hz) if dt is None else dt
        gb = self.acilis_g + _gauss_markov(n, self.bias_g, self.tau_s, dt, self.rng)
        ab = self.acilis_a + _gauss_markov(n, self.bias_a, self.tau_s, dt, self.rng)
        gn = self.rng.normal(0.0, self.sigma_g, (n, 3)) if self.sigma_g > 0 \
            else np.zeros((n, 3))
        an = self.rng.normal(0.0, self.sigma_a, (n, 3)) if self.sigma_a > 0 \
            else np.zeros((n, 3))
        return gb, ab, gn, an

    def uygula(self, gyro, ivme, zaman_s):
        """Ham olculeri bozar ve yonelim hatasini hesaplar.

        gyro, ivme : (N,3) temiz olcumler
        zaman_s    : (N,) ornek zamanlari (saniye, sim saati)

        donus: (gyro_bozuk, ivme_bozuk, d_rpy) — d_rpy (N,3) yonelim hatasi
               [d_roll, d_pitch, d_yaw] radyan.
        """
        gyro = np.asarray(gyro, dtype=float)
        ivme = np.asarray(ivme, dtype=float)
        zaman_s = np.asarray(zaman_s, dtype=float)
        n = len(gyro)
        dt_dizi = np.diff(zaman_s, prepend=zaman_s[0] - 1.0 / self.hiz_hz)
        # zaman atlamalarina karsi koru (bag'de bosluk olabilir)
        dt_dizi = np.clip(dt_dizi, 0.0, 10.0 / self.hiz_hz)
        dt_ort = float(np.median(dt_dizi)) or (1.0 / self.hiz_hz)

        gb, ab, gn, an = self.iz_uret(n, dt_ort)
        gyro_b = gyro * (1.0 + self.olcek_g) + gb + gn
        ivme_b = ivme * (1.0 + self.olcek_a) + ab + an

        # roll/pitch: ivmeolcer referansli, sinirli hata (kucuk aci)
        d_roll = ab[:, 1] / G
        d_pitch = -ab[:, 0] / G
        # yaw: gyro-z entegrasyonu, sinirsiz suruklenme
        yaw_hizi_hatasi = gyro_b[:, 2] - gyro[:, 2]
        d_yaw = np.cumsum(yaw_hizi_hatasi * dt_dizi)
        return gyro_b, ivme_b, np.stack([d_roll, d_pitch, d_yaw], axis=1)


# ------------------------------------------------------------------ kuaternion
def quat_carp(q1, q2):
    """Hamilton carpimi, (x,y,z,w) duzeni."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_to_rpy(q):
    x, y, z, w = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sin_p = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_p)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def yonelim_boz(q_temiz, d_rpy):
    """Temiz yonelime hata uygular.

    Yaw suruklenmesi DUNYA dikeyinde, roll/pitch hatasi GOVDE cercevesinde
    etki eder:   q_bozuk = Rz(d_yaw) * q_temiz * Rrp(d_roll, d_pitch)
    """
    d_roll, d_pitch, d_yaw = d_rpy
    q_yaw = rpy_to_quat(0.0, 0.0, d_yaw)
    q_rp = rpy_to_quat(d_roll, d_pitch, 0.0)
    q = quat_carp(q_yaw, quat_carp(np.asarray(q_temiz, dtype=float), q_rp))
    return q / np.linalg.norm(q)
