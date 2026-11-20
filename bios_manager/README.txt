build:

gcc -O2 -Wall -Wextra -Werror /root/bios-manager-q8b-v2.c -o /usr/local/sbin/bios-manager

usage:

  bios-manager -l
  bios-manager -s ПОЗИЦИЯ УСТРОЙСТВО [-n] [-y]

Параметры:
  -l              показать порядок загрузки
  -s N DEVICE     переместить первый DEVICE на позицию N
  -n              только показать результат, ничего не записывать
  -y              записать без подтверждения
  -h              справка

Устройства: SD, USB, NVMe, eMMC, UFS
Пример: bios-manager -s 1 NVMe -n

Q8B: USB (ID 12) может восстановить замененную ранее запись UFS.
Записи 7 и 8 сохраняются без изменений; назначение неизвестно.
