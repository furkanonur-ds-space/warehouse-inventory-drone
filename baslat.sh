#!/bin/bash
echo "🧹 Eski processler temizleniyor..."
pkill -f mavsdk_server
pkill -f swarm_drone
pkill -f master_swarm
pkill -f server.py
sleep 2

echo "📡 İstihbarat sunucusu başlatılıyor..."
python server.py &
sleep 2

echo "🚁🚁 İki drone aynı anda başlatılıyor..."
python master_swarm.py
