#!/bin/bash
# ─── TÜM ARUCO TESTLERİNİ SIRAYLA ÇALIŞTIRIR ───────────────
# Kullanım: bash run_all_tests.sh
# Her test için PX4'ü yeniden başlatır ve sonuçları tek CSV'ye yazar

SCRIPT_DIR=~/autonomous_landing
VENV=$SCRIPT_DIR/venv/bin/activate
TEST_SCRIPT=$SCRIPT_DIR/aruco_test.py
CSV=$SCRIPT_DIR/aruco_test_results.csv
PX4_DIR=~/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4

# Önceki sonuçları temizle
rm -f $CSV

echo "======================================================"
echo "  ARUCO KAPSAMLI TEST SUITEI BASLIYOR"
echo "  Toplam test kombinasyonu: 15 (5 ID x 3 boyut)"
echo "  Her kombinasyon: 6 senaryo (3 irtifa + 3 aci)"
echo "======================================================"

source $VENV

# Test kombinasyonları: "MARKER_ID MARKER_SIZE LIGHT"
TESTS=(
    "0  0.2 normal"
    "0  0.5 normal"
    "0  1.0 normal"
    "1  0.5 normal"
    "5  0.5 normal"
    "23 0.5 normal"
    "42 0.5 normal"
    "0  0.5 dark"
    "0  0.5 bright"
)

for TEST in "${TESTS[@]}"; do
    ID=$(echo $TEST | awk '{print $1}')
    SIZE=$(echo $TEST | awk '{print $2}')
    LIGHT=$(echo $TEST | awk '{print $3}')

    echo ""
    echo "──────────────────────────────────────────────────────"
    echo "  Siradaki test: ID=$ID  Boyut=${SIZE}m  Isik=$LIGHT"
    echo "──────────────────────────────────────────────────────"

    # PX4 SITL başlat (arka planda)
    cd $PX4_DIR
    HEADLESS=1 make px4_sitl gz_x500 > /tmp/px4_log.txt 2>&1 &
    PX4_PID=$!

    # PX4'ün hazır olmasını bekle
    echo "  [BEKLE] PX4 baslatiliyor..."
    sleep 15

    # Test scriptini çalıştır
    echo "  [CALISTIR] aruco_test.py $ID $SIZE $LIGHT"
    cd $SCRIPT_DIR
    python3 $TEST_SCRIPT $ID $SIZE $LIGHT

    # PX4'ü kapat
    kill $PX4_PID 2>/dev/null
    pkill -f gz 2>/dev/null
    pkill -f px4 2>/dev/null
    sleep 5

    echo "  [TAMAM] Test tamamlandi."
done

echo ""
echo "======================================================"
echo "  TUM TESTLER TAMAMLANDI!"
echo "  CSV dosyasi: $CSV"
echo "======================================================"

# Özet göster
echo ""
echo "SONUC OZETI:"
python3 -c "
import csv
results = []
with open('$CSV') as f:
    reader = csv.DictReader(f)
    for row in reader:
        results.append(row)

print(f'Toplam kayit: {len(results)}')
detected = [r for r in results if r[\"detected\"] == \"1\"]
print(f'Basarili tespit: {len(detected)}/{len(results)} ({len(detected)/len(results)*100:.1f}%)')
"
