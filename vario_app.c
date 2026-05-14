/**
 * @file vario_app.c
 * @brief Вариометр для Flipper Zero на базе датчика BME280.
 *
 * Архитектура приложения:
 *   ┌─────────────────────────────────────────────────┐
 *   │  Главный поток (vario_app)                      │
 *   │  • Цикл событий GUI (furi_message_queue_get)    │
 *   │  • Обработка кнопок                             │
 *   │  • Управление подсветкой                        │
 *   └──────────────────┬──────────────────────────────┘
 *                      │ FuriMutex (защита VarioData)
 *   ┌──────────────────┴──────────────────────────────┐
 *   │  Рабочий поток (vario_worker)                   │
 *   │  • Чтение BME280 каждые 200 мс                  │
 *   │  • Фильтр Калмана → высота и вертикальная скорость│
 *   │  • Звуковая индикация (неблокирующая)           │
 *   │  • Запись CSV лога на SD карту                  │
 *   └─────────────────────────────────────────────────┘
 *
 * Кнопки:
 *   Вверх      — вкл/выкл звук
 *   Вниз       — вкл/выкл принудительную подсветку
 *   OK         — открыть меню настроек
 *   Назад      — закрыть меню / выйти из приложения
 *
 * Лог пишется на SD карту в /ext/vario_log_YYYYMMDD.csv
 */

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>
#include <dialogs/dialogs.h>
#include <datetime/datetime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bme280.h"

#define TAG "Vario"

/* ── Параметры фильтра Калмана ────────────────────────────────────────── */

/* Дисперсия шума измерений давления (Па²).
 * Меньше → фильтр доверяет датчику, быстрее реагирует, но шумнее. */
#define KF_VAR_MEASUREMENT 0.1f

/* Дисперсия ускорения (Па²/с⁴).
 * Больше → фильтр быстрее отслеживает изменения давления. */
#define KF_VAR_ACCEL       0.75f

/* Давление на уровне моря в Паскалях (стандартная атмосфера). */
#define SEA_LEVEL_PRESSURE 101325.0f

/* ── Параметры звукового сигнала ──────────────────────────────────────── */

/* Порог вертикальной скорости подъёма для начала звукового сигнала (м/с). */
#define VARIO_CLIMB_THRESHOLD  0.3f

/* Порог вертикальной скорости снижения для начала звукового сигнала (м/с). */
#define VARIO_SINK_THRESHOLD  -0.9f

/* ── Настройки приложения (изменяются через меню) ─────────────────────── */
typedef struct {
    bool     sound_enabled;    /* Звуковая индикация включена             */
    bool     backlight_forced; /* Подсветка горит постоянно               */
    bool     logging_enabled;  /* Запись данных в CSV файл                */
    uint32_t log_interval;     /* Интервал записи в файл, секунды         */
} VarioSettings;

/* ── Общее состояние приложения ───────────────────────────────────────── */
typedef struct {
    /* --- Состояние фильтра Калмана ---
     * Фильтр оценивает давление (kf_x_abs, Па) и его производную
     * (kf_x_vel, Па/с). Из производной вычисляется вертикальная скорость. */
    float kf_x_abs;     /* Оценка текущего давления (Па)              */
    float kf_x_vel;     /* Оценка скорости изменения давления (Па/с)  */
    float kf_p_abs_abs; /* Дисперсия ошибки оценки давления           */
    float kf_p_abs_vel; /* Ковариация ошибок давления и скорости      */
    float kf_p_vel_vel; /* Дисперсия ошибки оценки скорости           */
    float kf_var_accel; /* Рабочая копия KF_VAR_ACCEL                 */

    /* --- Данные датчика и производные --- */
    float pressure;     /* Текущее давление, Па                       */
    float temperature;  /* Текущая температура, °C                    */
    float altitude;     /* Текущая высота, м                          */
    float vario;        /* Мгновенная вертикальная скорость, см/с     */
    float varioS;       /* Сглаженная вертикальная скорость, см/с     */

    /* --- Датчик --- */
    bool    sensor_ready; /* true = датчик инициализирован и работает  */
    BME280* bme280;       /* Дескриптор датчика (выделяется динамически)*/

    /* --- Настройки и UI --- */
    VarioSettings settings;
    uint32_t last_log_time;    /* Тик последней записи в файл           */
    uint32_t last_button_time; /* Тик последнего нажатия кнопки         */
    bool     show_menu;        /* true = показываем меню настроек        */
    uint8_t  menu_selection;   /* Выбранный пункт меню (0..3)           */

    /* --- Межпоточная синхронизация --- */
    FuriMutex*    mutex;   /* Защищает все поля структуры               */
    /* volatile: компилятор не кешируёт в регистре при -O2 оптимизации.
     * Читается из двух потоков без mutex — поэтому volatile обязателен. */
    volatile bool running; /* false = воркер должен завершиться         */
} VarioData;

/* Глобальный указатель — единственный экземпляр состояния приложения */
static VarioData*        vario_data   = NULL;
static NotificationApp*  notification = NULL;

/* ── Вспомогательные функции ──────────────────────────────────────────── */

/**
 * Пересчитывает давление в высоту по барометрической формуле.
 * @param pressure Давление в Паскалях.
 * @return Высота в метрах относительно уровня моря.
 */
static float pressure2altitude(float pressure) {
    /* Международная барометрическая формула:
     * h = 44330 × (1 − (P / P0)^0.190295) */
    return 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190295f));
}

/**
 * Сбрасывает фильтр Калмана в начальное состояние.
 * Вызывать при инициализации или переинициализации датчика.
 * @param abs_value Начальное давление (обычно SEA_LEVEL_PRESSURE).
 * @param vel_value Начальная скорость изменения давления (обычно 0).
 */
static void kf_reset(float abs_value, float vel_value) {
    vario_data->kf_x_abs     = abs_value;
    vario_data->kf_x_vel     = vel_value;
    vario_data->kf_p_abs_abs = 1.0e9f;   /* Большая неопределённость в начале */
    vario_data->kf_p_abs_vel = 0.0f;
    vario_data->kf_p_vel_vel = KF_VAR_ACCEL;
    vario_data->kf_var_accel = KF_VAR_ACCEL;
}

/* ── Инициализация датчика ────────────────────────────────────────────── */

/**
 * Выделяет память, инициализирует BME280 и настраивает параметры измерений.
 * При повторном вызове освобождает старый дескриптор.
 * Пробует оба I2C адреса (0x76 и 0x77).
 * @return true при успехе.
 */
static bool bme280_init_sensor(void) {
    /* Освобождаем старый дескриптор при повторной инициализации */
    if(vario_data->bme280) {
        free(vario_data->bme280);
        vario_data->bme280 = NULL;
    }

    vario_data->bme280 = malloc(sizeof(BME280));
    if(!vario_data->bme280) {
        FURI_LOG_E(TAG, "malloc failed");
        return false;
    }
    memset(vario_data->bme280, 0, sizeof(BME280));

    /* Пробуем адрес 0x76, затем 0x77 */
    bool ok = bme280_init(vario_data->bme280, BME280_I2C_ADDRESS_1,
                          &furi_hal_i2c_handle_external);
    if(!ok) {
        FURI_LOG_I(TAG, "0x76 failed, trying 0x77...");
        ok = bme280_init(vario_data->bme280, BME280_I2C_ADDRESS_2,
                         &furi_hal_i2c_handle_external);
    }
    if(!ok) {
        FURI_LOG_E(TAG, "BME280 not found on I2C");
        free(vario_data->bme280);
        vario_data->bme280 = NULL;
        return false;
    }

    /* Настройка параметров измерений:
     *   - IIR фильтр 16 — сильное сглаживание скачков давления (ветер, вибрация)
     *   - Standby 125 мс — датчик обновляет данные каждые ~125 мс
     *   - Оверсэмплинг температуры ×2 — достаточно для компенсации P и H
     *   - Оверсэмплинг давления ×4   — баланс точности и скорости
     *   - Оверсэмплинг влажности ×1  — нам важно давление, влажность вторична
     *   - Режим NORMAL — непрерывные измерения (set_mode вызывается последним) */
    bme280_set_filter(vario_data->bme280, BME280_FILTER_16);
    bme280_set_standby(vario_data->bme280, BME280_STANDBY_125);
    bme280_set_temp_oversample(vario_data->bme280, BME280_OS_2X);
    bme280_set_press_oversample(vario_data->bme280, BME280_OS_4X);
    bme280_set_humid_oversample(vario_data->bme280, BME280_OS_1X);
    bme280_set_mode(vario_data->bme280, BME280_MODE_NORMAL);

    FURI_LOG_I(TAG, "BME280 configured OK");
    return true;
}

/* ── Логирование на SD карту ──────────────────────────────────────────── */

/**
 * Записывает одну строку данных в CSV файл на SD карте.
 * Ничего не делает, если логирование выключено или интервал не истёк.
 * Данные снимаются под mutex, файловые операции — без mutex.
 */
static void log_to_file(void) {
    if(!vario_data->settings.logging_enabled) return;

    uint32_t now = furi_get_tick();
    /* Проверяем интервал (в миллисекундах) */
    if(now - vario_data->last_log_time < vario_data->settings.log_interval * 1000u)
        return;

    /* Снимаем локальные копии под mutex, чтобы не держать его во время I/O */
    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);
    float alt    = vario_data->altitude;
    float varioS = vario_data->varioS;
    float temp   = vario_data->temperature;
    float press  = vario_data->pressure;
    furi_mutex_release(vario_data->mutex);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    File*    file    = storage_file_alloc(storage);

    DateTime dt;
    furi_hal_rtc_get_datetime(&dt);

    /* Имя файла содержит дату — каждый день новый файл */
    char filename[64];
    snprintf(filename, sizeof(filename),
             "/ext/vario_log_%04d%02d%02d.csv",
             dt.year, dt.month, dt.day);

    if(storage_file_open(file, filename, FSAM_WRITE, FSOM_OPEN_APPEND)) {
        /* Пишем заголовок только в пустой файл */
        if(storage_file_size(file) == 0) {
            const char* hdr = "Time,Altitude_m,VarioS_ms,Temperature_C,Pressure_Pa\n";
            storage_file_write(file, hdr, strlen(hdr));
        }

        char line[128];
        snprintf(line, sizeof(line),
                 "%02d:%02d:%02d,%.2f,%.2f,%.1f,%.0f\n",
                 dt.hour, dt.minute, dt.second,
                 (double)alt,
                 (double)(varioS * 0.01f), /* см/с → м/с */
                 (double)temp,
                 (double)press);

        storage_file_write(file, line, strlen(line));
        storage_file_close(file);
    } else {
        FURI_LOG_E(TAG, "Cannot open log file: %s", filename);
    }

    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);

    vario_data->last_log_time = now;
}

/* ── Фильтр Калмана и расчёт высоты ──────────────────────────────────── */

/* Время последнего вызова update_pressure (секунды).
 * static — живёт всё время работы приложения. */
static float last_time_kf = 0.0f;

/**
 * Читает давление с датчика, прогоняет через фильтр Калмана,
 * вычисляет высоту и вертикальную скорость.
 * Вызывается из рабочего потока каждые 200 мс.
 */
static void update_pressure(void) {
    if(!vario_data->sensor_ready) return;

    float temp, press, humid;
    if(!bme280_read_sensor(vario_data->bme280, &temp, &press, &humid)) {
        FURI_LOG_W(TAG, "Sensor read failed");
        return;
    }

    /* Берём временну́ю метку ДО захвата mutex — I2C уже завершён */
    float t_now = (float)furi_get_tick() / 1000.0f; /* мс → с */
    float dt    = t_now - last_time_kf;
    last_time_kf = t_now;

    /* Защищаем от аномального dt на первом вызове или после пропуска тика */
    if(dt <= 0.0f || dt > 1.0f) dt = 0.2f;

    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);

    vario_data->temperature = temp;
    vario_data->pressure    = press;

    /* ── Предсказание (Predict step) ─────────────────────────────────
     * Модель: давление изменяется с постоянной скоростью kf_x_vel.
     * x̂_abs ← x̂_abs + x̂_vel × dt                                    */
    vario_data->kf_x_abs += vario_data->kf_x_vel * dt;

    /* Обновление матрицы ковариации ошибки:
     * P ← F × P × Fᵀ + Q
     * где F = [[1, dt],[0, 1]], Q — шум процесса от ускорения.       */
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    vario_data->kf_p_abs_abs += 2.0f * dt * vario_data->kf_p_abs_vel
                              + dt2   * vario_data->kf_p_vel_vel
                              + vario_data->kf_var_accel * dt4 / 4.0f;
    vario_data->kf_p_abs_vel += dt    * vario_data->kf_p_vel_vel
                              + vario_data->kf_var_accel * dt3 / 2.0f;
    vario_data->kf_p_vel_vel += vario_data->kf_var_accel * dt2;

    /* ── Коррекция (Update step) ──────────────────────────────────────
     * Инновация: разница между измерением и предсказанием.
     * Коэффициент Калмана K = P × Hᵀ × (H × P × Hᵀ + R)⁻¹            */
    float y     = press - vario_data->kf_x_abs; /* Инновация            */
    float s_inv = 1.0f / (vario_data->kf_p_abs_abs + KF_VAR_MEASUREMENT);
    float k_abs = vario_data->kf_p_abs_abs * s_inv; /* Усиление для давления */
    float k_vel = vario_data->kf_p_abs_vel * s_inv; /* Усиление для скорости */

    /* Обновление оценки состояния */
    vario_data->kf_x_abs += k_abs * y;
    vario_data->kf_x_vel += k_vel * y;

    /* Обновление матрицы ковариации: P ← (I − K × H) × P */
    vario_data->kf_p_vel_vel -= vario_data->kf_p_abs_vel * k_vel;
    vario_data->kf_p_abs_vel -= vario_data->kf_p_abs_vel * k_abs;
    vario_data->kf_p_abs_abs -= vario_data->kf_p_abs_abs * k_abs;

    /* ── Расчёт высоты и вертикальной скорости ────────────────────────
     * Высота из отфильтрованного давления.
     * Вертикальная скорость = разность высот при давлении P и P−dP/dt.
     * Результат в сантиметрах/сек (×100) для целочисленного удобства.  */
    float altitude = pressure2altitude(vario_data->kf_x_abs);
    vario_data->altitude = altitude;
    vario_data->vario    = (altitude -
                            pressure2altitude(vario_data->kf_x_abs - vario_data->kf_x_vel))
                           * 100.0f; /* м/с → см/с */

    /* Экспоненциальное сглаживание (IIR): α = 0.3 */
    vario_data->varioS = 0.7f * vario_data->varioS + 0.3f * vario_data->vario;

    furi_mutex_release(vario_data->mutex);
}

/* ── Звуковая индикация ───────────────────────────────────────────────── */

/**
 * Неблокирующее управление динамиком.
 *
 * Логика:
 *   - При подъёме (vspeed > CLIMB_THRESHOLD):  короткие частые бипы,
 *     частота растёт с ростом скорости (1300..2300 Гц).
 *   - При снижении (vspeed < SINK_THRESHOLD):  низкий непрерывный тон,
 *     частота падает с ростом скорости снижения (300..1000 Гц).
 *   - В нейтральной зоне: тишина.
 *
 * Реализация неблокирующая: звук запускается и сразу же функция
 * возвращается. Остановка происходит в следующей итерации по таймеру,
 * не блокируя рабочий поток.
 */
static void sound_update(void) {
    if(!vario_data || !vario_data->sensor_ready || !vario_data->settings.sound_enabled) return;

    static uint32_t next_beep_time = 0;  /* Тик начала следующего бипа   */
    static uint32_t beep_end_time  = 0;  /* Тик окончания текущего бипа  */

    uint32_t now = furi_get_tick();

    /* Останавливаем звук по истечении beep_end_time.
     * acquire(0) — не ждём: если динамик наш, получим true мгновенно. */
    if(beep_end_time != 0 && now >= beep_end_time) {
        if(furi_hal_speaker_acquire(0)) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
        }
        beep_end_time = 0;
    }

    /* Ещё не время следующего бипа */
    if(now < next_beep_time) return;

    /* Читаем вертикальную скорость под mutex */
    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);
    float vspeed = vario_data->varioS * 0.01f; /* см/с → м/с */
    furi_mutex_release(vario_data->mutex);

    if(vspeed > VARIO_CLIMB_THRESHOLD) {
        /* ── Подъём: короткий бип, частота и частота повторения растут ─ */
        /* beep_period: от 300 мс при малом подъёме до 50 мс при быстром */
        uint32_t beep_ms = (uint32_t)(350 - (int)(vspeed * 50.0f));
        if(beep_ms < 50u) beep_ms = 50u;

        uint32_t gap_ms  = beep_ms / 8u; /* Пауза = 1/8 длительности бипа */

        /* Частота: от 1300 Гц до 2300 Гц */
        int freq = 1300 + (int)(vspeed * 100.0f);
        if(freq > 2300) freq = 2300;

        next_beep_time = now + beep_ms + gap_ms;
        beep_end_time  = now + beep_ms;

        if(furi_hal_speaker_acquire(0)) {
            notification_message(notification, &sequence_set_only_blue_255);
            furi_hal_speaker_start((float)freq, 1.0f);
            /* Остановка произойдёт в следующем вызове по beep_end_time */
        }

    } else if(vspeed < VARIO_SINK_THRESHOLD) {
        /* ── Снижение: тон ниже и дольше с ростом скорости снижения ─── */
        /* beep_period: от 300 мс при малом снижении до 50 мс при быстром */
        int bp = 350 + (int)(vspeed * 25.0f); /* vspeed отрицательная */
        if(bp < 50) bp = 50;
        uint32_t beep_ms = (uint32_t)bp;
        uint32_t gap_ms  = beep_ms / 8u;

        /* Частота: от 1000 Гц до 300 Гц */
        int freq = 1000 + (int)(vspeed * 50.0f);
        if(freq < 300) freq = 300;

        next_beep_time = now + beep_ms + gap_ms;
        beep_end_time  = now + beep_ms;

        if(furi_hal_speaker_acquire(0)) {
            notification_message(notification, &sequence_set_only_red_255);
            furi_hal_speaker_start((float)freq, 1.0f);
        }

    } else {
        /* ── Нейтральная зона: тишина, сброс LED ─────────────────────── */
        notification_message(notification, &sequence_reset_blue);
        notification_message(notification, &sequence_reset_red);
        next_beep_time = now + 100u; /* Проверяем снова через 100 мс */
    }
}

/* ── Управление подсветкой ────────────────────────────────────────────── */

/**
 * Включает подсветку если:
 *   а) включена принудительно в настройках, или
 *   б) нажималась кнопка менее 5 секунд назад.
 * Работает через state machine: посылает уведомление только при смене состояния.
 */
static void update_backlight(void) {
    static bool last_state = false;

    bool active = vario_data->settings.backlight_forced ||
                  (furi_get_tick() - vario_data->last_button_time < 5000u);

    if(active != last_state) {
        if(active) notification_message(notification, &sequence_display_backlight_on);
        last_state = active;
    }
}

/* ── Отрисовка GUI ────────────────────────────────────────────────────── */

/**
 * Callback отрисовки — вызывается GUI потоком Flipper при каждом
 * обновлении экрана. Всегда захватывает mutex перед чтением данных.
 */
static void draw_callback(Canvas* canvas, void* ctx) {
    UNUSED(ctx);

    /* Защита от вызова до инициализации */
    if(!vario_data) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 30, "Initializing...");
        return;
    }

    /* Захватываем mutex на всё время отрисовки.
     * Это гарантирует, что данные не изменятся посередине кадра. */
    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);

    /* ── Экран меню настроек ──────────────────────────────────────────── */
    if(vario_data->show_menu) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 10, "Settings");
        canvas_draw_line(canvas, 0, 12, 128, 12);

        canvas_set_font(canvas, FontSecondary);

        /* Пункт 0: звук */
        if(vario_data->menu_selection == 0)
            canvas_draw_frame(canvas, 8, 14, 50, 12); /* рамка вокруг активного */
        canvas_draw_str(canvas, 10, 22, "Sound:");
        canvas_draw_str(canvas, 50, 22, vario_data->settings.sound_enabled ? "ON" : "OFF");

        /* Пункт 1: подсветка */
        if(vario_data->menu_selection == 1)
            canvas_draw_frame(canvas, 8, 26, 60, 12);
        canvas_draw_str(canvas, 10, 34, "Backlight:");
        canvas_draw_str(canvas, 60, 34, vario_data->settings.backlight_forced ? "ON" : "OFF");

        /* Пункт 2: логирование */
        if(vario_data->menu_selection == 2)
            canvas_draw_frame(canvas, 8, 38, 50, 12);
        canvas_draw_str(canvas, 10, 46, "Logging:");
        canvas_draw_str(canvas, 50, 46, vario_data->settings.logging_enabled ? "ON" : "OFF");

        /* Пункт 3: интервал логирования */
        if(vario_data->menu_selection == 3)
            canvas_draw_frame(canvas, 8, 50, 60, 12);
        canvas_draw_str(canvas, 10, 58, "Interval:");
        char ival[10];
        snprintf(ival, sizeof(ival), "%us", (unsigned)vario_data->settings.log_interval);
        canvas_draw_str(canvas, 60, 58, ival);

        furi_mutex_release(vario_data->mutex);
        return;
    }

    /* ── Экран "датчик не найден" ─────────────────────────────────────── */
    if(!vario_data->sensor_ready) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 5, 20, "BME280 not found!");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 5, 35, "Check I2C wiring:");
        canvas_draw_str(canvas, 5, 45, "SCL=C0  SDA=C1");
        canvas_draw_str(canvas, 5, 55, "VCC=3.3V  GND=GND");
        furi_mutex_release(vario_data->mutex);
        return;
    }

    /* ── Основной экран вариометра ────────────────────────────────────── */
    char buf[32];

    /* Иконки статуса (верхний левый и правый углы) */
    canvas_set_font(canvas, FontSecondary);
    if(!vario_data->settings.sound_enabled)
        canvas_draw_str(canvas, 2, 8, "MUTE");
    if(vario_data->settings.logging_enabled)
        canvas_draw_str(canvas, 105, 8, "LOG");

    /* Стрелка подъёма / снижения */
    float vspeed = vario_data->varioS * 0.01f; /* см/с → м/с */
    canvas_set_font(canvas, FontPrimary);
    if(vspeed > 0.1f)       canvas_draw_str(canvas, 2, 18, "^");
    else if(vspeed < -0.1f) canvas_draw_str(canvas, 2, 18, "v");

    /* Высота крупным шрифтом по центру */
    canvas_set_font(canvas, FontBigNumbers);
    snprintf(buf, sizeof(buf), "%.0f", (double)vario_data->altitude);
    canvas_draw_str_aligned(canvas, 64, 18, AlignCenter, AlignBottom, buf);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 30, AlignCenter, AlignTop, "m");

    /* Вертикальная скорость */
    canvas_set_font(canvas, FontPrimary);
    if(fabsf(vspeed) < 0.01f)
        snprintf(buf, sizeof(buf), "0.00 m/s");
    else
        snprintf(buf, sizeof(buf), "%+.2f m/s", (double)vspeed); /* + показывает знак */
    canvas_draw_str_aligned(canvas, 64, 40, AlignCenter, AlignTop, buf);

    /* Температура и давление в нижней строке */
    canvas_set_font(canvas, FontSecondary);
    snprintf(buf, sizeof(buf), "%.1fC", (double)vario_data->temperature);
    canvas_draw_str(canvas, 2, 55, buf);

    snprintf(buf, sizeof(buf), "%.0fhPa", (double)(vario_data->pressure / 100.0f));
    canvas_draw_str_aligned(canvas, 126, 55, AlignRight, AlignTop, buf);

    furi_mutex_release(vario_data->mutex);
}

/* ── Обработка кнопок ─────────────────────────────────────────────────── */

/**
 * Input callback — вызывается GUI потоком при нажатии кнопки.
 * Просто кладёт событие в очередь главного потока.
 * Никакой бизнес-логики здесь — только передача события.
 */
static void handle_input(InputEvent* event, void* context) {
    FuriMessageQueue* queue = (FuriMessageQueue*)context;
    furi_message_queue_put(queue, event, 0);
}

/* ── Рабочий поток ────────────────────────────────────────────────────── */

/**
 * Основной рабочий поток: инициализация датчика, цикл измерений.
 * Завершается когда vario_data->running становится false.
 */
static int32_t vario_worker(void* context) {
    UNUSED(context);

    /* Инициализируем настройки. Делаем это здесь (не в vario_app),
     * чтобы иметь чёткое начальное состояние до первого отображения. */
    vario_data->settings.sound_enabled    = true;
    vario_data->settings.backlight_forced = false;
    vario_data->settings.logging_enabled  = false;
    vario_data->settings.log_interval     = 10; /* секунд */

    vario_data->last_log_time    = 0;
    vario_data->last_button_time = furi_get_tick();
    vario_data->menu_selection   = 0;
    vario_data->varioS           = 0.0f;

    FURI_LOG_I(TAG, "Worker started, initializing BME280...");

    if(bme280_init_sensor()) {
        vario_data->sensor_ready = true;
        last_time_kf = (float)furi_get_tick() / 1000.0f;
        kf_reset(SEA_LEVEL_PRESSURE, 0.0f);
        FURI_LOG_I(TAG, "BME280 ready");
    } else {
        vario_data->sensor_ready = false;
        FURI_LOG_E(TAG, "BME280 not found, will retry every 5s");
    }

    uint32_t last_retry_tick = 0;

    while(vario_data->running) {
        if(vario_data->sensor_ready) {
            update_pressure(); /* Чтение датчика + фильтр Калмана (~5..15 мс) */
            sound_update();    /* Неблокирующее управление динамиком           */
            log_to_file();     /* Запись в файл (только если интервал истёк)   */
        } else {
            /* Пробуем переинициализировать датчик каждые 5 секунд */
            uint32_t now = furi_get_tick();
            if(now - last_retry_tick > 5000u) {
                last_retry_tick = now;
                FURI_LOG_I(TAG, "Retrying BME280 init...");
                if(bme280_init_sensor()) {
                    vario_data->sensor_ready = true;
                    last_time_kf = (float)furi_get_tick() / 1000.0f;
                    kf_reset(SEA_LEVEL_PRESSURE, 0.0f);
                    FURI_LOG_I(TAG, "BME280 reconnected");
                }
            }
        }

        /* 200 мс = ~5 обновлений в секунду.
         * Соответствует standby датчика 125 мс + время одного измерения. */
        furi_delay_ms(200);
    }

    FURI_LOG_I(TAG, "Worker stopped");
    return 0;
}

/* ── Точка входа приложения ───────────────────────────────────────────── */

/**
 * Точка входа. Имя должно совпадать с entry_point в application.fam.
 * Запускает GUI, рабочий поток, обрабатывает события до нажатия Back.
 */
int32_t vario_app(void* p) {
    UNUSED(p);

    /* Очередь событий от кнопок (max 8 событий, тип InputEvent) */
    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    /* Выделяем и обнуляем структуру состояния */
    vario_data = malloc(sizeof(VarioData));
    memset(vario_data, 0, sizeof(VarioData));
    vario_data->mutex   = furi_mutex_alloc(FuriMutexTypeNormal);
    vario_data->running = true;

    /* Регистрируем viewport — экран приложения */
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, NULL);
    /* context = event_queue: handle_input положит события в очередь */
    view_port_input_callback_set(view_port, handle_input, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    notification = furi_record_open(RECORD_NOTIFICATION);

    /* Запускаем рабочий поток.
     * Стек 4096 байт: float-математика + I2C + snprintf для лога. */
    FuriThread* worker = furi_thread_alloc();
    furi_thread_set_name(worker, "VarioWorker");
    furi_thread_set_stack_size(worker, 4096);
    furi_thread_set_context(worker, vario_data);
    furi_thread_set_callback(worker, vario_worker);
    furi_thread_start(worker);

    /* ── Главный цикл событий ─────────────────────────────────────────── */
    InputEvent event;
    while(vario_data->running) {
        /* Ждём событие не более 100 мс, затем обновляем подсветку и экран */
        if(furi_message_queue_get(event_queue, &event, 100) == FuriStatusOk) {
            furi_mutex_acquire(vario_data->mutex, FuriWaitForever);
            vario_data->last_button_time = furi_get_tick();

            /* Back вне меню = выход из приложения */
            if(event.type == InputTypeShort && event.key == InputKeyBack
               && !vario_data->show_menu) {
                vario_data->running = false;
                furi_mutex_release(vario_data->mutex);
                break;
            }

            if(event.type == InputTypeShort) {
                if(vario_data->show_menu) {
                    /* ── Навигация в меню ──────────────────────────────── */
                    switch(event.key) {
                    case InputKeyUp:
                        /* Циклический переход вверх (4 пункта: 0..3) */
                        vario_data->menu_selection = (vario_data->menu_selection + 3u) % 4u;
                        break;
                    case InputKeyDown:
                        vario_data->menu_selection = (vario_data->menu_selection + 1u) % 4u;
                        break;
                    case InputKeyOk:
                        /* Переключение выбранного параметра */
                        switch(vario_data->menu_selection) {
                        case 0:
                            vario_data->settings.sound_enabled ^= true;
                            break;
                        case 1:
                            vario_data->settings.backlight_forced ^= true;
                            break;
                        case 2:
                            vario_data->settings.logging_enabled ^= true;
                            break;
                        case 3:
                            /* Циклическое изменение интервала логирования */
                            if(vario_data->settings.log_interval == 1u)
                                vario_data->settings.log_interval = 2u;
                            else if(vario_data->settings.log_interval == 2u)
                                vario_data->settings.log_interval = 5u;
                            else if(vario_data->settings.log_interval == 5u)
                                vario_data->settings.log_interval = 10u;
                            else if(vario_data->settings.log_interval == 10u)
                                vario_data->settings.log_interval = 30u;
                            else if(vario_data->settings.log_interval == 30u)
                                vario_data->settings.log_interval = 60u;
                            else
                                vario_data->settings.log_interval = 1u;
                            break;
                        default:
                            break;
                        }
                        break;
                    case InputKeyBack:
                        /* Back в меню = закрыть меню, НЕ выходить */
                        vario_data->show_menu = false;
                        break;
                    default:
                        break;
                    }
                } else {
                    /* ── Быстрые действия на основном экране ──────────── */
                    switch(event.key) {
                    case InputKeyUp:
                        vario_data->settings.sound_enabled ^= true;
                        break;
                    case InputKeyDown:
                        vario_data->settings.backlight_forced ^= true;
                        break;
                    case InputKeyOk:
                        vario_data->show_menu      = true;
                        vario_data->menu_selection = 0;
                        break;
                    default:
                        break;
                    }
                }
            }

            furi_mutex_release(vario_data->mutex);
            view_port_update(view_port);
        }

        update_backlight();
        view_port_update(view_port);
    }

    /* ── Завершение работы ────────────────────────────────────────────── */

    vario_data->running = false; /* На случай если вышли не через break */

    /* Останавливаем звук: acquire(0) — не ждём, просто проверяем */
    if(furi_hal_speaker_acquire(0)) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    /* Гасим LED (могли остаться включёнными от sound_update) */
    notification_message(notification, &sequence_reset_blue);
    notification_message(notification, &sequence_reset_red);

    /* Ждём завершения рабочего потока, только потом освобождаем память.
     * furi_thread_free без join — undefined behavior. */
    furi_thread_join(worker);
    furi_thread_free(worker);

    /* Отключаем viewport до удаления GUI */
    view_port_enabled_set(view_port, false);
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    /* Освобождаем ресурсы */
    furi_mutex_free(vario_data->mutex);
    if(vario_data->bme280) free(vario_data->bme280);
    free(vario_data);
    vario_data = NULL;

    furi_message_queue_free(event_queue);

    return 0;
}
