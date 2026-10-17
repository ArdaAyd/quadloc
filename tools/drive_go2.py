#!/usr/bin/env python3
"""Go2'yi yurutur, kosuyu canli izler ve gecerli olup olmadigina karar verir.

    drive_go2.py [sure_sim_s] [ly] [--json CIKTI] [--rota ROTA]

ROTA verilirse robot serbest duz yurumek yerine tanimli bir yoru izler ve
rotayi bitirince durur. Kullanilabilir rotalar:

    duz:12            ileri 12 m
    gidis_donus:12    ileri 12 m, don, baslangica gel
    kare:8            8x8 m kare tur
    zigzag:12:3       12 m boyunca 3 m genliginde zikzak

Rota takibi, robotun yonunu simulatorun referans konumundan duzelterek
calisir. Bu mesru: otonom gezinme projenin kapsami disinda (rapor §3) ve
amac yorungenin kosular arasinda **ayni** kalmasi. Ayni rota her kosulda
tekrarlanmazsa, SLAM hatasindaki fark yorunge farkindan mi kosul farkindan
mi geldigi ayirt edilemez.

Sure SIMULASYON saniyesi cinsindendir, duvar saati degil: simulasyon gercek
zamanin ~0.6 kati hizinda ilerledigi icin duvar saatiyle olculen bir pencere
kosudan kosuya farkli uzunlukta veri uretirdi.

Cikis kodlari (tekrar mekanizmasi bunlara bakar):
    0  kosu gecerli
    2  robot devrildi
    3  robot ilerlemedi (takildi)
    4  kurulum basarisiz (kontrolcu yok, ground truth gelmiyor, FSM gecmedi)
"""
import sys, os, time, math, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_input_msgs.msg import Inputs
from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry

GECERLI, DEVRILDI, TAKILDI, KURULUM = 0, 2, 3, 4

# Esikler olculen yuruyusten turetildi: saglikli yuruyuste govde 0.32-0.39 m
# arasinda, |roll| genelde 0.45 rad altinda kaliyor. Kayitli bir kosuda roll
# 0.65 rad'a (-37 derece) cikip kendini toparladi, bu yuzden esik 1.2 rad:
# gercek devrilmeler 2-3 rad'a gidiyor, toparlanabilen salinimlar degil.
# Hepsi ortam degiskeniyle ezilebilir (GO2_DEVRILME_ROLL=0.5 gibi): esikleri
# yeni ortamlarda ayarlamak ve tespit mekanizmasini sinamak icin.
def _esik(ad, varsayilan):
    try:
        return float(os.environ.get(ad, varsayilan))
    except ValueError:
        return varsayilan

DEVRILME_ROLL   = _esik("GO2_DEVRILME_ROLL", 1.2)     # rad
DEVRILME_PITCH  = _esik("GO2_DEVRILME_PITCH", 1.2)    # rad
DEVRILME_YUKSEK = _esik("GO2_DEVRILME_YUKSEK", 0.20)  # m
DEVRILME_SURE   = _esik("GO2_DEVRILME_SURE", 0.4)     # s (sim)
TAKILMA_PENCERE = _esik("GO2_TAKILMA_PENCERE", 8.0)   # s (sim)
TAKILMA_MESAFE  = _esik("GO2_TAKILMA_MESAFE", 0.3)    # m


# Rota takip kazanclari. Donus hizi siniri +-0.5 rad/s oldugu icin kazanc
# yuksek tutulursa doygunluga girip salinim yapiyor; 1.5 olcumle secildi.
ROTA_KP_YAW   = 1.5
ROTA_VARIS_M  = 0.6    # bu mesafeye girilince bir sonraki hedefe gecilir
ROTA_YAVASLA  = 0.25   # yonelim bozukken ileri hizin dusurulecegi taban carpan


def rota_uret(ad):
    """'gidis_donus:12' gibi bir metinden hedef noktalar listesi uretir."""
    parcalar = ad.split(":")
    tur = parcalar[0]
    sayi = [float(x) for x in parcalar[1:]] or [12.0]
    m = sayi[0]
    if tur == "duz":
        return [(m, 0.0)]
    if tur == "gidis_donus":
        return [(m, 0.0), (0.0, 0.0)]
    if tur == "kare":
        return [(m, 0.0), (m, m), (0.0, m), (0.0, 0.0)]
    if tur == "zigzag":
        genlik = sayi[1] if len(sayi) > 1 else 3.0
        adim = m / 4.0
        return [(adim, genlik), (2*adim, -genlik), (3*adim, genlik), (m, 0.0)]
    raise ValueError(f"bilinmeyen rota: {ad}")


def aci_sar(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class RotaTakipci:
    """Hedef noktalari sirayla izler; (ly, rx) komutu uretir."""

    def __init__(self, noktalar, p0, yaw0, ly_maks):
        c, sn = math.cos(yaw0), math.sin(yaw0)
        # Rota, robotun baslangic konumuna ve yonune gore tanimlanir; boylece
        # ayni rota her dunyada ayni sekilde kosulur.
        self.hedefler = [(p0[0] + c*x - sn*y, p0[1] + sn*x + c*y) for x, y in noktalar]
        self.i = 0
        self.ly_maks = ly_maks
        self.tamam = False

    def hedef(self):
        return self.hedefler[self.i] if self.i < len(self.hedefler) else None

    def komut(self, pose, yaw):
        h = self.hedef()
        if h is None:
            self.tamam = True
            return 0.0, 0.0
        dx, dy = h[0] - pose[0], h[1] - pose[1]
        uzaklik = math.hypot(dx, dy)
        if uzaklik < ROTA_VARIS_M:
            self.i += 1
            if self.i >= len(self.hedefler):
                self.tamam = True
                return 0.0, 0.0
            return self.komut(pose, yaw)
        yon_hatasi = aci_sar(math.atan2(dy, dx) - yaw)
        # Kontrolcude d_yaw_cmd_ = -invNormalize(rx, -0.5, 0.5): pozitif rx
        # saat yonunde donduruyor, yani yaw'i artirmak icin rx negatif olmali.
        rx = max(-1.0, min(1.0, -ROTA_KP_YAW * yon_hatasi))
        # Yonelim bozukken yavasla, once don
        ly = self.ly_maks * max(ROTA_YAVASLA, math.cos(yon_hatasi))
        return ly, rx


def quat_rpy(q):
    roll  = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
    yaw   = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return roll, pitch, yaw


class Driver(Node):
    def __init__(self):
        super().__init__("go2_driver")
        self.pub = self.create_publisher(Inputs, "/control_input", 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, qos)
        self.state = "passive"
        self.pose = self.rpy = self.vel = None
        self.t_sim = None          # simulasyon saati (odom basliklarindan)
        self.gecisler = []

    def _on_log(self, m):
        if "Switched from" in m.msg:
            self.state = m.msg.split(" to ")[-1].strip()
            self.gecisler.append(self.state)
            print(f"      [FSM] {m.msg}", flush=True)

    def _on_odom(self, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.pose = (p.x, p.y, p.z)
        self.rpy = quat_rpy(m.pose.pose.orientation)
        self.vel = (v.x, v.y, v.z)
        self.t_sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    # --- temel islemler ---
    def spin(self, secs):
        """Duvar saatiyle bekler: ROS kesfi ve FSM gecisleri icin kullanilir."""
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, command=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        m = Inputs()
        m.command, m.lx, m.ly, m.rx, m.ry = command, lx, ly, rx, ry
        self.pub.publish(m)

    def hazir_mi(self, timeout=40):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.pub.get_subscription_count() > 0 and self.pose is not None:
                return True
            self.spin(0.2)
        return False

    def gec(self, command, hedef, sim_timeout=20.0, duvar_timeout=300.0):
        """Hedef FSM durumuna gecene kadar komutu araliklarla yeniler.

        FSM gecisleri simulasyon zamaninda ilerledigi icin (her gecis ~1.8 s sim)
        zaman asimi da simulasyon zamanindan olculur. Agir dunyalarda gercek zaman
        carpani 0.07'ye kadar dustugu icin duvar saatine bagli bir sinir, robot
        gayet saglikliyken "gecilemedi" hatasi veriyordu. Duvar saati siniri
        yalnizca simulasyon tamamen durduysa devreye giren emniyet kemeri."""
        t_sim0 = self.t_sim
        duvar0 = time.time()
        while True:
            if hedef in self.state:
                return True
            if self.t_sim is not None and t_sim0 is not None and self.t_sim - t_sim0 > sim_timeout:
                return False
            if time.time() - duvar0 > duvar_timeout:
                return False
            self.send(command=command)
            self.spin(0.4)
            for _ in range(14):
                self.send(command=0)
                self.spin(0.15)
                if hedef in self.state:
                    return True

    def otur(self, en_fazla_sim=5.0):
        """Govde duruncaya kadar bekler (simulasyon zamaninda). Tirisa erken
        girmek devrilmenin baslica sebebiydi: StateTrotting::enter() hedef
        yuksekligi o anki ayak-govde mesafesinden aliyor."""
        t_sim0 = self.t_sim
        duvar0 = time.time()
        while self.t_sim - t_sim0 < en_fazla_sim:
            self.send(command=0)
            self.spin(0.1)
            if (self.vel and math.hypot(*self.vel) < 0.01
                    and self.t_sim - t_sim0 > 1.5):
                break
            if time.time() - duvar0 > 300:
                break
        return self.t_sim - t_sim0


class Gozcu:
    """Kosuyu canli izler; devrilme veya takilmayi olustugu anda yakalar."""
    def __init__(self, t0_sim, p0):
        self.t0 = t0_sim
        self.p0 = p0
        self.kotu_baslangic = None
        self.gecmis = [(t0_sim, p0)]
        self.maks_roll = self.maks_pitch = 0.0
        self.min_z = 9.9

    def guncelle(self, t_sim, pose, rpy):
        self.maks_roll = max(self.maks_roll, abs(rpy[0]))
        self.maks_pitch = max(self.maks_pitch, abs(rpy[1]))
        self.min_z = min(self.min_z, pose[2])

        kotu = (abs(rpy[0]) > DEVRILME_ROLL or abs(rpy[1]) > DEVRILME_PITCH
                or pose[2] < DEVRILME_YUKSEK)
        if kotu:
            if self.kotu_baslangic is None:
                self.kotu_baslangic = t_sim
            elif t_sim - self.kotu_baslangic >= DEVRILME_SURE:
                return DEVRILDI
        else:
            self.kotu_baslangic = None

        self.gecmis.append((t_sim, pose))
        while self.gecmis and t_sim - self.gecmis[0][0] > TAKILMA_PENCERE:
            self.gecmis.pop(0)
        if t_sim - self.t0 > TAKILMA_PENCERE and len(self.gecmis) > 1:
            t_eski, p_eski = self.gecmis[0]
            if math.hypot(pose[0] - p_eski[0], pose[1] - p_eski[1]) < TAKILMA_MESAFE:
                return TAKILDI
        return GECERLI


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    json_yolu = None
    rota_adi = None
    for a in sys.argv[1:]:
        if a.startswith("--json="):
            json_yolu = a.split("=", 1)[1]
        elif a.startswith("--rota="):
            rota_adi = a.split("=", 1)[1]
    sure = float(argv[0]) if argv else 30.0        # SIM saniyesi
    ly   = float(argv[1]) if len(argv) > 1 else 1.0

    rclpy.init()
    d = Driver()
    sonuc = {"istenen_sure_sim_s": sure, "ly": ly, "komut_hiz_ms": round(0.4 * ly, 3),
             "rota": rota_adi or "serbest"}

    def bitir(kod, not_=""):
        sonuc["sonuc"] = {GECERLI: "gecerli", DEVRILDI: "devrildi",
                          TAKILDI: "takildi", KURULUM: "kurulum_hatasi"}[kod]
        sonuc["not"] = not_
        print("SONUC_JSON " + json.dumps(sonuc, ensure_ascii=False), flush=True)
        if json_yolu:
            with open(json_yolu, "w") as f:
                json.dump(sonuc, f, indent=2, ensure_ascii=False)
        rclpy.shutdown()
        sys.exit(kod)

    if not d.hazir_mi():
        bitir(KURULUM, "/control_input abonesi veya /ground_truth/odom yok")

    for cmd, hedef in ((2, "fixed down"), (2, "fixed stand"), (4, "trotting")):
        if hedef == "trotting":
            bekleme = d.otur()
            print(f"  oturdu ({bekleme:.1f} s sim)", flush=True)
        if not d.gec(cmd, hedef):
            bitir(KURULUM, f"'{hedef}' durumuna gecilemedi (son durum: {d.state})")
        print(f"  -> {hedef}", flush=True)

    d.spin(0.5)
    t0, p0 = d.t_sim, d.pose
    g = Gozcu(t0, p0)

    takipci = None
    if rota_adi:
        try:
            noktalar = rota_uret(rota_adi)
        except ValueError as e:
            bitir(KURULUM, str(e))
        takipci = RotaTakipci(noktalar, p0, d.rpy[2], ly)
        yol = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                  for a, b in zip([p0[:2]] + takipci.hedefler, takipci.hedefler))
        sonuc["rota_uzunlugu_m"] = round(yol, 2)
        print(f"\n-> rota '{rota_adi}': {len(noktalar)} hedef, toplam {yol:.1f} m, "
              f"en fazla {sure:.0f} s simulasyon zamani", flush=True)
    else:
        print(f"\n-> yuruyus: ly={ly} ({0.4*ly:.2f} m/s), {sure:.0f} s simulasyon zamani",
              flush=True)

    durum = GECERLI
    isaret = 5.0
    duvar0 = time.time()
    while d.t_sim - t0 < sure:
        if takipci is None:
            d.send(command=0, ly=ly)
        else:
            if d.pose is not None:
                k_ly, k_rx = takipci.komut(d.pose, d.rpy[2])
                if takipci.tamam:
                    break
                d.send(command=0, ly=k_ly, rx=k_rx)
        d.spin(0.02)
        if d.pose is None:
            continue
        durum = g.guncelle(d.t_sim, d.pose, d.rpy)
        gecen = d.t_sim - t0
        if gecen >= isaret:
            ek = ""
            if takipci is not None and takipci.hedef():
                hx, hy = takipci.hedef()
                ek = (f"  hedef {takipci.i+1}/{len(takipci.hedefler)} "
                      f"({math.hypot(hx-d.pose[0], hy-d.pose[1]):4.1f} m)")
            print(f"  t={gecen:5.1f}s  x={d.pose[0]:+7.3f} y={d.pose[1]:+6.3f} "
                  f"z={d.pose[2]:.3f}  roll={d.rpy[0]:+5.2f}{ek}", flush=True)
            isaret += 5.0
        if durum != GECERLI:
            break
        # Yalnizca simulasyon tamamen durduysa devreye girer; agir dunyalarda
        # gercek zaman carpani 0.07'ye kadar dusebiliyor, bu yuzden pay genis.
        if time.time() - duvar0 > 40 * sure + 300:
            bitir(KURULUM, "simulasyon zamani ilerlemiyor")

    gecen_sim = d.t_sim - t0
    mesafe = math.hypot(d.pose[0] - p0[0], d.pose[1] - p0[1])
    sonuc.update({
        "sure_sim_s": round(gecen_sim, 2),
        "sure_duvar_s": round(time.time() - duvar0, 2),
        "rtf": round(gecen_sim / max(time.time() - duvar0, 1e-6), 3),
        "mesafe_m": round(mesafe, 3),
        "hiz_ms": round(mesafe / gecen_sim, 3) if gecen_sim > 0.5 else 0.0,
        "yanal_sapma_m": round(abs(d.pose[1] - p0[1]), 3),
        "maks_roll_rad": round(g.maks_roll, 3),
        "maks_pitch_rad": round(g.maks_pitch, 3),
        "min_govde_z_m": round(g.min_z, 3),
    })
    print(f"\n>>> mesafe {mesafe:.2f} m / {gecen_sim:.1f} s sim = "
          f"{sonuc['hiz_ms']:.3f} m/s  (komut {0.4*ly:.2f})", flush=True)
    print(f">>> RTF {sonuc['rtf']:.2f}   maks roll {g.maks_roll:.2f} rad   "
          f"min z {g.min_z:.3f} m", flush=True)

    if takipci is not None:
        sonuc["rota_tamamlandi"] = bool(takipci.tamam)
        sonuc["ulasilan_hedef"] = takipci.i
        sonuc["hedef_sayisi"] = len(takipci.hedefler)

    if durum == DEVRILDI:
        bitir(DEVRILDI, f"t={gecen_sim:.1f}s sim'de devrildi")
    if durum == TAKILDI:
        bitir(TAKILDI, f"{TAKILMA_PENCERE:.0f} s icinde {TAKILMA_MESAFE} m ilerlemedi")
    if takipci is not None and not takipci.tamam:
        # Rota bitmeden sure doldu: kayit kullanilabilir ama yorunge eksik,
        # kosullar arasi karsilastirma icin ayni rotanin tamamlanmasi gerek.
        bitir(TAKILDI, f"rota tamamlanamadi: {takipci.i}/{len(takipci.hedefler)} "
                       f"hedefe ulasildi, sure doldu")
    bitir(GECERLI)


if __name__ == "__main__":
    main()
