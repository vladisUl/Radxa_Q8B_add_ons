Q8B FAN PI v2 — тихое журналирование

Изменения относительно v1:
- В штатном режиме CSV НЕ создаётся, периодические строки температуры убраны.
- В /var/log/q8b-fan/events.log пишутся только запуск/остановка,
  смена OFF/PI/PROTECT, применение конфигурации и ошибки.
- Те же события видны в терминале и через journalctl, если запущено systemd.
- --record включает подробный CSV старого формата, интервал log_period_s.
- Калибровка теперь тоже не создаёт CSV без --record.
- ПИ, датчики, пороги защиты, режимы PWM, сторожевой процесс не изменялись.
- Никакого кольцевого буфера нет.

Сборка (из распакованного каталога q8b_fan_v2):
    gcc -std=c11 -O2 -Wall -Wextra -Werror q8b-fan.c -lm -o q8b-fan

Ручной запуск:
    ./q8b-fan --run ./q8b-fan.conf

По желанию, подробная запись температур:
    ./q8b-fan --run ./q8b-fan.conf --record

Допустимо также: ./q8b-fan --run --record ./q8b-fan.conf
При --record CSV появляется в /var/log/q8b-fan/q8b-fan-YYYYMMDD-PID.csv.
Старые CSV версия v2 не удаляет.

Калибровка:
    ./q8b-fan --calibrate ./q8b-fan.conf

Управление/инспекция:
    ./q8b-fan --probe
    ./q8b-fan --restore   (только если контроллер не работает)
    tail -n 30 /var/log/q8b-fan/events.log

Для установки в systemd:
    install -m 755 q8b-fan /usr/local/sbin/q8b-fan
    install -m 644 q8b-fan.service /etc/systemd/system/q8b-fan.service
    systemctl daemon-reload
    systemctl start q8b-fan
    journalctl -u q8b-fan -n 30 --no-pager

ВНИМАНИЕ: не запускайте две копии программы одновременно. Если v1 уже
работает, сначала остановите её Ctrl+C или systemctl stop q8b-fan.
Если у вас уже настроен /etc/q8b-fan.conf — НЕ заменяйте его шаблоном: 
уставка, ПИ и пороги защиты сохранены без изменений и совместимы с v2.
Новые параметры в конфигурации НЕ требуются.

На SIGINT/SIGTERM контроллер возвращает ADSP auto: pwm1_enable=2.
В режиме --record ошибка записи CSV трактуется как ошибка программы,
без --record зависимость от CSV полностью отсутствует.
