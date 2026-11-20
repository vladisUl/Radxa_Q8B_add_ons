#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/fs.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* Q8B: exact vendor GUID and 52-byte layout. Support observed duplicated-UFS or USB-restored layout. */
#ifndef EFI_GLOB
#define EFI_GLOB "/sys/firmware/efi/efivars/BootDevOrder-882f8c2b-9646-435f-8de5-f208ff80c1bd"
#endif
#define SLOT_COUNT 12
#define ACTIVE_COUNT 6
#define VAR_SIZE (4 + SLOT_COUNT * 4)

struct device_map {
    uint32_t id;
    const char *name;
};

static const struct device_map devices[] = {
    { 9,  "SD"   },
    { 12, "USB"  },
    { 6,  "NVMe" },
    { 2,  "eMMC" },
    { 11, "UFS"  },
};

static void usage(FILE *out)
{
    fprintf(out,
        "Использование:\n"
        "  bios-manager -l\n"
        "  bios-manager -s ПОЗИЦИЯ УСТРОЙСТВО [-n] [-y]\n"
        "\n"
        "Параметры:\n"
        "  -l              показать порядок загрузки\n"
        "  -s N DEVICE     переместить первый DEVICE на позицию N\n"
        "  -n              только показать результат, ничего не записывать\n"
        "  -y              записать без подтверждения\n"
        "  -h              справка\n"
        "\n"
        "Устройства: SD, USB, NVMe, eMMC, UFS\n"
        "Пример: bios-manager -s 1 NVMe -n\n"
        "\nQ8B: USB (ID 12) может восстановить замененную ранее запись UFS.\n"
        "Записи 7 и 8 сохраняются без изменений; назначение неизвестно.\n");
}

static void die(const char *msg)
{
    fprintf(stderr, "Ошибка: %s\n", msg);
    exit(EXIT_FAILURE);
}

static void die_errno(const char *what)
{
    fprintf(stderr, "Ошибка: %s: %s\n", what, strerror(errno));
    exit(EXIT_FAILURE);
}

static uint32_t get_le32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static const char *device_name(uint32_t id)
{
    size_t i;
    for (i = 0; i < sizeof(devices) / sizeof(devices[0]); ++i)
        if (devices[i].id == id)
            return devices[i].name;
    return NULL;
}

static int device_id(const char *name, uint32_t *id)
{
    size_t i;
    for (i = 0; i < sizeof(devices) / sizeof(devices[0]); ++i) {
        if (strcasecmp(name, devices[i].name) == 0) {
            *id = devices[i].id;
            return 0;
        }
    }
    return -1;
}

static char *find_variable(void)
{
    glob_t g;
    char *result;
    int rc = glob(EFI_GLOB, 0, NULL, &g);

    if (rc == GLOB_NOMATCH)
        die("переменная BootDevOrder не найдена");
    if (rc != 0)
        die("ошибка поиска BootDevOrder");
    if (g.gl_pathc != 1) {
        globfree(&g);
        die("найдено несколько переменных BootDevOrder");
    }

    result = strdup(g.gl_pathv[0]);
    globfree(&g);
    if (!result)
        die_errno("strdup");
    return result;
}

static size_t read_all(const char *path, uint8_t *buf, size_t cap)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    size_t total = 0;

    if (fd < 0)
        die_errno("не удалось открыть BootDevOrder");

    while (total < cap) {
        ssize_t n = read(fd, buf + total, cap - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            close(fd);
            die_errno("не удалось прочитать BootDevOrder");
        }
        if (n == 0)
            break;
        total += (size_t)n;
    }

    close(fd);
    return total;
}

static int write_all_fd(int fd, const uint8_t *buf, size_t size)
{
    size_t total = 0;

    while (total < size) {
        ssize_t n = write(fd, buf + total, size - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0) {
            errno = EIO;
            return -1;
        }
        total += (size_t)n;
    }
    return 0;
}

static int write_variable_once(const char *path, const uint8_t *buf, size_t size)
{
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    int saved_errno;

    if (fd < 0)
        return -1;

    if (write_all_fd(fd, buf, size) < 0) {
        saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }

    if (fsync(fd) < 0 && errno != EINVAL && errno != EOPNOTSUPP) {
        saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }

    if (close(fd) < 0)
        return -1;
    return 0;
}

static int get_file_flags(const char *path, int *flags)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    int rc;
    int saved_errno;

    if (fd < 0)
        return -1;
    rc = ioctl(fd, FS_IOC_GETFLAGS, flags);
    saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return rc;
}

static int set_file_flags(const char *path, int flags)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    int rc;
    int saved_errno;

    if (fd < 0)
        return -1;
    rc = ioctl(fd, FS_IOC_SETFLAGS, &flags);
    saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return rc;
}

static void parse_order(const uint8_t raw[VAR_SIZE], uint32_t order[SLOT_COUNT])
{
    int i;
    for (i = 0; i < SLOT_COUNT; ++i)
        order[i] = get_le32(raw + 4 + i * 4);
}

static void encode_order(uint8_t raw[VAR_SIZE], const uint32_t order[SLOT_COUNT])
{
    int i;
    for (i = 0; i < SLOT_COUNT; ++i)
        put_le32(raw + 4 + i * 4, order[i]);
}

/* Q8B observed layouts:
 * A: 6,7,8,9,11,11 (USB was replaced by a second UFS in BIOS)
 * B: 6,7,8,9,11,12 (USB restored).
 * Do not accept other IDs or counts for writing.
 */
static void validate_order(const uint32_t order[SLOT_COUNT])
{
    int seen_zero = 0;
    int active = 0;
    int count6 = 0, count7 = 0, count8 = 0, count9 = 0, count11 = 0, count12 = 0;
    int i;

    for (i = 0; i < SLOT_COUNT; ++i) {
        if (order[i] == 0) {
            seen_zero = 1;
            continue;
        }
        if (seen_zero)
            die("в BootDevOrder найдено ненулевое значение после NONE");
        ++active;
        switch (order[i]) {
        case 6: ++count6; break;
        case 7: ++count7; break;
        case 8: ++count8; break;
        case 9: ++count9; break;
        case 11: ++count11; break;
        case 12: ++count12; break;
        default: die("Q8B: неизвестный идентификатор; запись запрещена");
        }
    }
    if (active != ACTIVE_COUNT || count6 != 1 || count7 != 1 ||
        count8 != 1 || count9 != 1 ||
        !((count11 == 2 && count12 == 0) ||
          (count11 == 1 && count12 == 1)))
        die("Q8B: состав BootDevOrder отличается от двух проверяемых профилей");
}

static void print_order(const uint32_t order[SLOT_COUNT])
{
    int i;
    for (i = 0; i < ACTIVE_COUNT; ++i) {
        const char *name = device_name(order[i]);
        if (name)
            printf("%d  %s (ID %u)\n", i + 1, name, order[i]);
        else
            printf("%d  ID %u (назначение не определено)\n", i + 1, order[i]);
    }
}

static void make_backup(const uint8_t raw[VAR_SIZE], char *out, size_t out_size)
{
    const char *base = "/var/lib/bios-manager";
    const char *dir = "/var/lib/bios-manager/backups";
    time_t now = time(NULL);
    struct tm tm_now;
    char stamp[32];
    int fd;

    if (mkdir(base, 0700) < 0 && errno != EEXIST)
        die_errno("не удалось создать /var/lib/bios-manager");
    if (mkdir(dir, 0700) < 0 && errno != EEXIST)
        die_errno("не удалось создать каталог резервных копий");

    if (!localtime_r(&now, &tm_now))
        die("не удалось получить текущее время");
    if (strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_now) == 0)
        die("не удалось сформировать имя резервной копии");

    if (snprintf(out, out_size, "%s/BootDevOrder-%s-%ld.bin",
                 dir, stamp, (long)getpid()) >= (int)out_size)
        die("слишком длинный путь резервной копии");

    fd = open(out, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0)
        die_errno("не удалось создать резервную копию");
    if (write_all_fd(fd, raw, VAR_SIZE) < 0) {
        int saved_errno = errno;
        close(fd);
        unlink(out);
        errno = saved_errno;
        die_errno("не удалось записать резервную копию");
    }
    if (fsync(fd) < 0) {
        int saved_errno = errno;
        close(fd);
        unlink(out);
        errno = saved_errno;
        die_errno("не удалось синхронизировать резервную копию");
    }
    if (close(fd) < 0)
        die_errno("не удалось закрыть резервную копию");
}

static int confirm_write(void)
{
    char line[32];
    printf("\nЗаписать новый порядок в UEFI? [y/N] ");
    fflush(stdout);
    if (!fgets(line, sizeof(line), stdin))
        return 0;
    return line[0] == 'y' || line[0] == 'Y';
}

static void apply_change(const char *path,
                         const uint8_t original[VAR_SIZE],
                         const uint8_t updated[VAR_SIZE],
                         const char *backup_path)
{
    int flags = 0;
    int immutable_cleared = 0;
    uint8_t verify[VAR_SIZE + 1];
    size_t verify_size;

    /* Refuse writes if another actor has modified the EFI variable meanwhile. */
    verify_size = read_all(path, verify, sizeof(verify));
    if (verify_size != VAR_SIZE || memcmp(verify, original, VAR_SIZE) != 0)
        die("BootDevOrder изменился после чтения; запись отменена");

    if (get_file_flags(path, &flags) == 0) {
        if (flags & FS_IMMUTABLE_FL) {
            int new_flags = flags & ~FS_IMMUTABLE_FL;
            if (set_file_flags(path, new_flags) < 0)
                die_errno("не удалось временно снять immutable с BootDevOrder");
            immutable_cleared = 1;
        }
    } else if (errno != ENOTTY && errno != EOPNOTSUPP && errno != ENOSYS) {
        die_errno("не удалось прочитать флаги BootDevOrder");
    }

    if (write_variable_once(path, updated, VAR_SIZE) < 0) {
        int saved_errno = errno;
        if (immutable_cleared)
            (void)set_file_flags(path, flags);
        errno = saved_errno;
        die_errno("запись BootDevOrder не удалась");
    }

    verify_size = read_all(path, verify, sizeof(verify));
    if (verify_size != VAR_SIZE || memcmp(verify, updated, VAR_SIZE) != 0) {
        fprintf(stderr, "Ошибка: проверка после записи не прошла, выполняю откат.\n");
        if (write_variable_once(path, original, VAR_SIZE) < 0) {
            fprintf(stderr,
                    "КРИТИЧЕСКАЯ ОШИБКА: откат не удался: %s\n"
                    "Резервная копия: %s\n",
                    strerror(errno), backup_path);
        } else {
            fprintf(stderr, "Исходное значение восстановлено.\n");
        }
        if (immutable_cleared && set_file_flags(path, flags) < 0)
            fprintf(stderr, "Предупреждение: не удалось вернуть immutable: %s\n",
                    strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (immutable_cleared && set_file_flags(path, flags) < 0)
        fprintf(stderr, "Предупреждение: не удалось вернуть immutable: %s\n",
                strerror(errno));
}

int main(int argc, char **argv)
{
    enum { MODE_NONE, MODE_LIST, MODE_SET } mode = MODE_NONE;
    int dry_run = 0;
    int assume_yes = 0;
    int position = 0;
    const char *requested_name = NULL;
    uint32_t requested_id = 0;
    char *path;
    uint8_t raw_read[VAR_SIZE + 1];
    uint8_t raw[VAR_SIZE];
    uint8_t updated[VAR_SIZE];
    uint32_t order[SLOT_COUNT];
    uint32_t new_order[SLOT_COUNT];
    size_t size;
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0) {
            if (mode != MODE_NONE)
                die("укажи только одну операцию: -l или -s");
            mode = MODE_LIST;
        } else if (strcmp(argv[i], "-s") == 0) {
            char *end = NULL;
            long value;
            if (mode != MODE_NONE)
                die("укажи только одну операцию: -l или -s");
            if (i + 2 >= argc)
                die("после -s нужны позиция и устройство");
            errno = 0;
            value = strtol(argv[++i], &end, 10);
            if (errno || !end || *end != '\0' || value < 1 || value > ACTIVE_COUNT)
                die("позиция должна быть от 1 до 6");
            position = (int)value;
            requested_name = argv[++i];
            if (device_id(requested_name, &requested_id) < 0)
                die("неизвестное устройство; доступны SD, USB, NVMe, eMMC, UFS");
            mode = MODE_SET;
        } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--dry-run") == 0) {
            dry_run = 1;
        } else if (strcmp(argv[i], "-y") == 0) {
            assume_yes = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(stdout);
            return 0;
        } else {
            usage(stderr);
            return EXIT_FAILURE;
        }
    }

    if (mode == MODE_NONE) {
        usage(stderr);
        return EXIT_FAILURE;
    }
    if (mode == MODE_LIST && (dry_run || assume_yes))
        die("-n и -y применимы только вместе с -s");

    path = find_variable();
    size = read_all(path, raw_read, sizeof(raw_read));
    if (size != VAR_SIZE) {
        fprintf(stderr, "Ошибка: размер BootDevOrder равен %zu байт, ожидалось %d\n",
                size, VAR_SIZE);
        free(path);
        return EXIT_FAILURE;
    }
    memcpy(raw, raw_read, VAR_SIZE);
    if (get_le32(raw) != 0x00000007u)
        die("Q8B: неожиданные атрибуты UEFI-переменной (ожидалось 0x00000007)");

    parse_order(raw, order);
    validate_order(order);

    if (mode == MODE_LIST) {
        printf("Порядок загрузки:\n\n");
        print_order(order);
        free(path);
        return 0;
    }

    memcpy(new_order, order, sizeof(new_order));

    {
        int old_index = -1;
        int target = position - 1;

        for (i = 0; i < ACTIVE_COUNT; ++i) {
            if (new_order[i] == requested_id) {
                old_index = i;
                break;
            }
        }

        /* User replaced USB (12) with UFS (11) in BIOS; there are two UFS
         * entries. When selecting USB again, restore the first UFS slot
         * to USB while retaining the second UFS. This is the only allowed
         * insertion, and the resulting six-entry profile is validated. */
        if (old_index < 0 && requested_id == 12u) {
            int first_ufs = -1, ufs_count = 0;
            for (i = 0; i < ACTIVE_COUNT; ++i) {
                if (new_order[i] == 11u) {
                    if (first_ufs < 0)
                        first_ufs = i;
                    ++ufs_count;
                }
            }
            if (ufs_count != 2 || first_ufs < 0)
                die("USB отсутствует, но заменить дублирующий UFS нельзя");
            new_order[first_ufs] = 12u;
            old_index = first_ufs;
            printf("Q8B: восстанавливаю USB (ID 12) вместо первого дублирующего UFS (ID 11).\n\n");
        }

        if (old_index < 0)
            die("устройство отсутствует в текущем BootDevOrder");

        if (old_index < target) {
            for (i = old_index; i < target; ++i)
                new_order[i] = new_order[i + 1];
        } else if (old_index > target) {
            for (i = old_index; i > target; --i)
                new_order[i] = new_order[i - 1];
        }
        new_order[target] = requested_id;
        validate_order(new_order);
    }

    printf("Текущий порядок:\n\n");
    print_order(order);
    printf("\nНовый порядок:\n\n");
    print_order(new_order);

    if (memcmp(order, new_order, sizeof(order)) == 0) {
        printf("\nИзменений нет.\n");
        free(path);
        return 0;
    }

    if (dry_run) {
        printf("\nРежим проверки: запись не выполнялась.\n");
        free(path);
        return 0;
    }

    if (geteuid() != 0)
        die("для изменения UEFI запусти bios-manager от root");

    if (!assume_yes && !confirm_write()) {
        printf("Отменено.\n");
        free(path);
        return 0;
    }

    memcpy(updated, raw, sizeof(updated));
    encode_order(updated, new_order);

    {
        char backup_path[512];
        make_backup(raw, backup_path, sizeof(backup_path));
        apply_change(path, raw, updated, backup_path);
        printf("\nГотово. Новый порядок записан и проверен.\n");
        printf("Резервная копия: %s\n", backup_path);
    }

    free(path);
    return 0;
}
