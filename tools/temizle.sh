#!/bin/bash
# Simulasyondan kalan butun surecleri oldurur.
#
# Neden ayri bir betik: once temizlik deseni her betige elle yazilmisti ve
# robot_state_publisher listede yoktu. Her launch birini geride biraktigi icin
# 75 surece kadar birikip DDS trafigini bogdular; gercek zaman carpani 0.6'dan
# 0.08'e dustu ve bacak odometrisi olcumlerini sessizce bozdu. Tek yerden
# yonetmek bunun tekrarini onluyor.
PATTERN="[g]z sim|[r]os2 launch|[r]obot_state_publisher|[q]uadruped_controller|[s]tate_estimation|[p]arameter_bridge|[i]mage_bridge|[e]kf_node|[f]oot_contacts_bridge|[s]tatic_transform_publisher|[c]ontroller_manager/spawner|[r]viz2|[r]os2 bag record|[r]uby .*gz sim"

ps aux | grep -E "$PATTERN" | awk '{print $2}' | xargs -r kill -9 2>/dev/null
# bayat paylasimli bellek dosyalari yeni kosularda DDS hatasi veriyor
rm -f /dev/shm/fastrtps_* 2>/dev/null
sleep "${1:-4}"

# grep kendi komut satirinda deseni tasidigi icin kendini saymasin
KALAN=$(ps aux | grep -E "$PATTERN" | grep -v "grep -E" | wc -l)
if [ "$KALAN" -gt 0 ]; then
  echo "UYARI: $KALAN surec hala ayakta" >&2
  ps aux | grep -E "$PATTERN" | grep -v "grep -E" | awk '{print "  "$2, $11}' >&2
fi
