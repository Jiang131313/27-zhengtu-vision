sleep 5
cd ~/Desktop/sp_vision_25_hero/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "AUTOSTART_LOG=1 ./fuck.sh"
