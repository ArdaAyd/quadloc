#!/bin/bash
# Kaygan zemin kosulunu kademelendirir ve her seviyede olcum toplar.
#
#   tools/surtunme_taramasi.sh [dunya] [rota] [deneme] [mu listesi...]
#   tools/surtunme_taramasi.sh koridor_dokulu duz:10 2 0.8 0.5 0.3 0.2 0.15 0.1
#
# Kaygan zemin, kayit sonrasi uygulanabilen TEK OLMAYAN kosul: surtunme
# fizigi temas cozumunde, simulatorde kosmak zorunlu. Bu yuzden duman/IMU/
# karanliktan farkli olarak her seviye ayri bir kosu.
#
# Kaymanin dogal olcusu BACAK ODOMETRISI HATASI: bacak odometrisi ayagin
# yerde kaymadigini varsayar, kaydikca mesafeyi OLDUGUNDAN FAZLA sayar.
# Yani ayni olcum hem kosulu kademelendiriyor hem kapsam maddesi 6'yi besliyor.
HERE=$(cd "$(dirname "$0")" && pwd)
DUNYA=${1:-koridor_dokulu}; ROTA=${2:-duz:12}; DENEME=${3:-2}; shift 3 2>/dev/null
# Olcum sabit bir noktadan baslar; aksi halde isinmanin goturdugu yer kosudan
# kosuya degisiyor ve olculen yol 3.1-5.5 m arasinda savruluyor.
BASLANGIC=${BASLANGIC:-3,0}
TEKRAR=${TEKRAR:-1}
MUS=${@:-0.8 0.6 0.4 0.3 0.2 0.15 0.1}
CIKTI=$HERE/../kayitlar/surtunme_taramasi.csv

[ -f "$CIKTI" ] || echo "zaman,dunya,rota,mu,sonuc,yol_gercek_m,hata_yuzde,yol_seyreltik_m,hata_seyreltik_yuzde,salinim_payi,hiz_ms,sure_sim_s,not" > $CIKTI

for MU in $MUS; do
 for T in $(seq 1 $TEKRAR); do
  echo "=== mu = $MU (tekrar $T/$TEKRAR) ==="
  RAPOR=/tmp/surtunme_$MU.json
  rm -f $RAPOR
  RAPOR=$RAPOR $HERE/champ_kosu.sh --rota $ROTA --mutlak --baslangic $BASLANGIC \
        --dunya $DUNYA --profil lidar --hiz 0.2 --surtunme $MU --deneme $DENEME
  KOD=$?
  python3 - "$RAPOR" "$CIKTI" "$DUNYA" "$ROTA" "$MU" "$KOD" <<'PYEOF'
import json, sys, csv, datetime, os
rapor, cikti, dunya, rota, mu, kod = sys.argv[1:7]
d = {}
if os.path.exists(rapor):
    try: d = json.load(open(rapor))
    except Exception: pass
sonuc = {"0": "gecerli", "2": "devrildi", "3": "takildi/tamamlanamadi",
         "4": "kurulum hatasi"}.get(kod, f"kod {kod}")
with open(cikti, "a", newline="") as f:
    csv.writer(f).writerow([
        datetime.datetime.now().isoformat(timespec="seconds"), dunya, rota, mu,
        sonuc, d.get("yol_gercek_m", ""), d.get("bacak_odom_hatasi_yuzde", ""),
        d.get("yol_gercek_seyreltik_m", ""),
        d.get("bacak_odom_hatasi_seyreltik_yuzde", ""),
        d.get("salinim_payi_yuzde", ""), d.get("ortalama_hiz_ms", ""),
        d.get("sure_sim_s", ""), d.get("not", "")])
print(f">>> mu={mu}: {sonuc}  yol(2Hz)={d.get('yol_gercek_seyreltik_m','-')} m  "
      f"hata(2Hz)={d.get('bacak_odom_hatasi_seyreltik_yuzde','-')}%  "
      f"salinim=%{d.get('salinim_payi_yuzde','-')}  hiz={d.get('ortalama_hiz_ms','-')} m/s")
PYEOF
 done
done

echo
echo "=== ozet ==="
column -s, -t $CIKTI
