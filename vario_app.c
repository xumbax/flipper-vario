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
 *   Вверх  — вкл/выкл звук
 *   Вниз   — вкл/выкл принудительную подсветку
 *   OK     — открыть меню настроек
 *   Назад  — закрыть меню / выйти из приложения
 *
 * Лог: /ext/vario_log_YYYYMMDD.csv
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

/* ── Фильтр Калмана ───────────────────────────────────────────────────── */
#define KF_VAR_MEASUREMENT 0.1f   /* Дисперсия шума измерений (Па²)      */
#define KF_VAR_ACCEL       0.75f  /* Дисперсия ускорения (Па²/с⁴)        */
#define SEA_LEVEL_PRESSURE 101325.0f

/* ── Пороги звуковой сигнализации ────────────────────────────────────── */
#define VARIO_CLIMB_THRESHOLD  0.3f   /* м/с — начало сигнала подъёма    */
#define VARIO_SINK_THRESHOLD  -0.9f   /* м/с — начало сигнала снижения   */

/* ── Индикатор вертикальной скорости (полоса слева) ──────────────────── */
#define VBAR_X       4     /* X левого края полосы                        */
#define VBAR_W       6     /* Ширина полосы (пикселей)                    */
#define VBAR_CY     32     /* Y центра полосы (нейтраль)                  */
#define VBAR_HALF   26     /* Полувысота шкалы (пикселей)                 */
#define VBAR_MAX_MS  5.0f  /* Скорость при полном отклонении (м/с)        */

/* ── Меню настроек ────────────────────────────────────────────────────── */
/*
 * Экран Flipper: 128×64 px.
 * Заголовок "Settings" (FontPrimary ~12px) + разделитель занимают до y=14.
 * Доступно для 4 пунктов: y=15..63 = 48px.
 *
 * FontSecondary: высота глифа ~8px, координата Y = baseline (нижний край).
 * Шаг строк STEP=12px: текст 8px + 4px зазор.
 * Рамка: 1px выше верхнего края глифа, 1px ниже baseline → высота 10px.
 *
 * MENU_FIRST_Y=25  — baseline первой строки
 * MENU_ROW_STEP=12 — шаг между строками
 *
 * Проверка (baseline / frame_top / frame_bottom):
 *   [0]: 25 / 16 / 25  ✓
 *   [1]: 37 / 28 / 37  ✓
 *   [2]: 49 / 40 / 49  ✓
 *   [3]: 61 / 52 / 61  ✓  (61 < 64 — в пределах экрана)
 */
#define MENU_FONT_H    8   /* Высота FontSecondary (пикселей)             */
#define MENU_ROW_STEP 12   /* Шаг между строками меню (пикселей)          */
#define MENU_FIRST_Y  25   /* Baseline первой строки меню                 */
#define MENU_FRAME_H  (MENU_FONT_H + 2)             /* Высота рамки       */
#define MENU_ROW_Y(i)     (MENU_FIRST_Y + (i) * MENU_ROW_STEP)
#define MENU_FRAME_TOP(i) (MENU_ROW_Y(i) - MENU_FONT_H - 1)

/* ── Настройки приложения ─────────────────────────────────────────────── */
typedef struct {
    bool     sound_enabled;
    bool     backlight_forced;
    bool     logging_enabled;
    uint32_t log_interval; /* секунд */
} VarioSettings;

/* ── Состояние приложения ─────────────────────────────────────────────── */
typedef struct {
    /* Фильтр Калмана */
    float kf_x_abs;
    float kf_x_vel;
    float kf_p_abs_abs;
    float kf_p_abs_vel;
    float kf_p_vel_vel;
    float kf_var_accel;

    /* Данные датчика */
    float pressure;
    float temperature;
    float altitude;
    float vario;
    float varioS;

    /* Датчик */
    bool    sensor_ready;
    BME280* bme280;

    /* UI */
    VarioSettings settings;
    uint32_t last_log_time;
    uint32_t last_button_time;
    bool     show_menu;
    uint8_t  menu_selection;

    /* Синхронизация */
    FuriMutex*    mutex;
    volatile bool running;
} VarioData;

static VarioData*       vario_data   = NULL;
static NotificationApp* notification = NULL;

/* ── Вспомогательные функции ──────────────────────────────────────────── */

static float pressure2altitude(float pressure) {
    return 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190295f));
}

static void kf_reset(float abs_value, float vel_value) {
    vario_data->kf_x_abs     = abs_value;
    vario_data->kf_x_vel     = vel_value;
    vario_data->kf_p_abs_abs = 1.0e9f;
    vario_data->kf_p_abs_vel = 0.0f;
    vario_data->kf_p_vel_vel = KF_VAR_ACCEL;
    vario_data->kf_var_accel = KF_VAR_ACCEL;
}

/* ── Инициализация датчика ────────────────────────────────────────────── */

static bool bme280_init_sensor(void) {
    if(vario_data->bme280) {
        free(vario_data->bme280);
        vario_data->bme280 = NULL;
    }
    vario_data->bme280 = malloc(sizeof(BME280));
    if(!vario_data->bme280) return false;
    memset(vario_data->bme280, 0, sizeof(BME280));

    bool ok = bme280_init(vario_data->bme280, BME280_I2C_ADDRESS_1,
                          &furi_hal_i2c_handle_external);
    if(!ok) {
        FURI_LOG_I(TAG, "0x76 failed, trying 0x77...");
        ok = bme280_init(vario_data->bme280, BME280_I2C_ADDRESS_2,
                         &furi_hal_i2c_handle_external);
    }
    if(!ok) {
        free(vario_data->bme280);
        vario_data->bme280 = NULL;
        return false;
    }
    bme280_set_filter(vario_data->bme280, BME280_FILTER_16);
    bme280_set_standby(vario_data->bme280, BME280_STANDBY_125);
    bme280_set_temp_oversample(vario_data->bme280, BME280_OS_2X);
    bme280_set_press_oversample(vario_data->bme280, BME280_OS_4X);
    bme280_set_humid_oversample(vario_data->bme280, BME280_OS_1X);
    bme280_set_mode(vario_data->bme280, BME280_MODE_NORMAL);
    FURI_LOG_I(TAG, "BME280 OK");
    return true;
}

/* ── Логирование ──────────────────────────────────────────────────────── */

static void log_to_file(void) {
    if(!vario_data->settings.logging_enabled) return;
    uint32_t now = furi_get_tick();
    if(now - vario_data->last_log_time < vario_data->settings.log_interval * 1000u) return;

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

    char filename[64];
    snprintf(filename, sizeof(filename), "/ext/vario_log_%04d%02d%02d.csv",
             dt.year, dt.month, dt.day);

    if(storage_file_open(file, filename, FSAM_WRITE, FSOM_OPEN_APPEND)) {
        if(storage_file_size(file) == 0) {
            const char* hdr = "Time,Altitude_m,VarioS_ms,Temperature_C,Pressure_Pa\n";
            storage_file_write(file, hdr, strlen(hdr));
        }
        char line[128];
        snprintf(line, sizeof(line), "%02d:%02d:%02d,%.2f,%.2f,%.1f,%.0f\n",
                 dt.hour, dt.minute, dt.second,
                 (double)alt, (double)(varioS * 0.01f), (double)temp, (double)press);
        storage_file_write(file, line, strlen(line));
        storage_file_close(file);
    }
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    vario_data->last_log_time = now;
}

/* ── Фильтр Калмана ───────────────────────────────────────────────────── */

static float last_time_kf = 0.0f;

static void update_pressure(void) {
    if(!vario_data->sensor_ready) return;

    float temp, press, humid;
    if(!bme280_read_sensor(vario_data->bme280, &temp, &press, &humid)) return;

    float t_now = (float)furi_get_tick() / 1000.0f;
    float dt    = t_now - last_time_kf;
    last_time_kf = t_now;
    if(dt <= 0.0f || dt > 1.0f) dt = 0.2f;

    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);

    vario_data->temperature = temp;
    vario_data->pressure    = press;

    /* Predict */
    vario_data->kf_x_abs += vario_data->kf_x_vel * dt;
    float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    vario_data->kf_p_abs_abs += 2.0f*dt*vario_data->kf_p_abs_vel
                              + dt2*vario_data->kf_p_vel_vel
                              + vario_data->kf_var_accel*dt4/4.0f;
    vario_data->kf_p_abs_vel += dt*vario_data->kf_p_vel_vel
                              + vario_data->kf_var_accel*dt3/2.0f;
    vario_data->kf_p_vel_vel += vario_data->kf_var_accel*dt2;

    /* Update */
    float y     = press - vario_data->kf_x_abs;
    float s_inv = 1.0f / (vario_data->kf_p_abs_abs + KF_VAR_MEASUREMENT);
    float k_abs = vario_data->kf_p_abs_abs * s_inv;
    float k_vel = vario_data->kf_p_abs_vel * s_inv;
    vario_data->kf_x_abs += k_abs * y;
    vario_data->kf_x_vel += k_vel * y;
    vario_data->kf_p_vel_vel -= vario_data->kf_p_abs_vel * k_vel;
    vario_data->kf_p_abs_vel -= vario_data->kf_p_abs_vel * k_abs;
    vario_data->kf_p_abs_abs -= vario_data->kf_p_abs_abs * k_abs;

    float altitude = pressure2altitude(vario_data->kf_x_abs);
    vario_data->altitude = altitude;
    vario_data->vario    = (altitude -
        pressure2altitude(vario_data->kf_x_abs - vario_data->kf_x_vel)) * 100.0f;
    vario_data->varioS   = 0.7f * vario_data->varioS + 0.3f * vario_data->vario;

    furi_mutex_release(vario_data->mutex);
}

/* ── Звуковая индикация ───────────────────────────────────────────────── */

/*
 * ИСПРАВЛЕНО — два критических бага:
 *
 * БАГ 1 (звук не останавливается / MUTE не работает):
 *   Предыдущая версия делала acquire→start, но НЕ делала release после start.
 *   Это означало что динамик оставался захваченным нами навсегда.
 *   При следующей попытке stop: acquire(0) возвращал false (мы сами держим),
 *   поэтому stop никогда не вызывался.
 *
 *   Правильная модель владения Flipper speaker:
 *     acquire → start → ... → stop → release   (всё в одной сессии)
 *   Для неблокирующего звука мы ДЕРЖИМ speaker между start и stop,
 *   используя статический флаг speaker_owned.
 *
 * БАГ 2 (звук не останавливается после выхода):
 *   При выходе speaker мог быть захвачен нами (speaker_owned=true).
 *   Теперь sound_stop() вызывается явно при завершении воркера.
 */

/* Статические переменные состояния звука.
 * static внутри функции — живут всё время работы воркера. */
static bool     sound_speaker_owned = false; /* мы держим speaker сейчас */
static uint32_t sound_next_beep     = 0;     /* тик начала следующего бипа */
static uint32_t sound_beep_end      = 0;     /* тик конца текущего бипа */

/* Безусловно останавливает динамик и освобождает его.
 * Вызывать при выключении звука и при завершении приложения. */
static void sound_stop(void) {
    if(sound_speaker_owned) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
        sound_speaker_owned = false;
    }
    sound_beep_end  = 0;
    sound_next_beep = 0;
}

static void sound_update(void) {
    /* Если звук выключен — немедленно глушим динамик если он наш */
    if(!vario_data || !vario_data->sensor_ready || !vario_data->settings.sound_enabled) {
        sound_stop();
        return;
    }

    uint32_t now = furi_get_tick();

    /* Конец бипа — останавливаем звук, но НЕ освобождаем speaker немедленно.
     * Освобождение происходит в sound_stop() или при следующем acquire. */
    if(sound_speaker_owned && sound_beep_end != 0 && now >= sound_beep_end) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
        sound_speaker_owned = false;
        sound_beep_end      = 0;
    }

    /* Ещё не время следующего бипа */
    if(now < sound_next_beep) return;

    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);
    float vspeed = vario_data->varioS * 0.01f; /* см/с → м/с */
    furi_mutex_release(vario_data->mutex);

    if(vspeed > VARIO_CLIMB_THRESHOLD) {
        /* Подъём: короткий бип, частота растёт со скоростью */
        uint32_t beep_ms = (uint32_t)(350 - (int)(vspeed * 50.0f));
        if(beep_ms < 50u) beep_ms = 50u;
        uint32_t gap_ms  = beep_ms / 4u;

        int freq = 1300 + (int)(vspeed * 100.0f);
        if(freq > 2300) freq = 2300;

        sound_next_beep = now + beep_ms + gap_ms;
        sound_beep_end  = now + beep_ms;

        /* acquire с таймаутом 10мс — если система занята, пробуем чуть подождать */
        if(furi_hal_speaker_acquire(10)) {
            sound_speaker_owned = true;
            furi_hal_speaker_start((float)freq, 1.0f);
            /* speaker остаётся захваченным до sound_beep_end */
        }

    } else if(vspeed < VARIO_SINK_THRESHOLD) {
        /* Снижение: низкий тон, частота падает со скоростью */
        int bp = 350 + (int)(vspeed * 25.0f);
        if(bp < 50) bp = 50;
        uint32_t beep_ms = (uint32_t)bp;
        uint32_t gap_ms  = beep_ms / 4u;

        int freq = 1000 + (int)(vspeed * 50.0f);
        if(freq < 300) freq = 300;

        sound_next_beep = now + beep_ms + gap_ms;
        sound_beep_end  = now + beep_ms;

        if(furi_hal_speaker_acquire(10)) {
            sound_speaker_owned = true;
            furi_hal_speaker_start((float)freq, 1.0f);
        }

    } else {
        /* Нейтральная зона: тишина */
        sound_next_beep = now + 100u;
        /* beep_end уже обнулён выше или равен 0 */
    }
}

/* ── Подсветка ────────────────────────────────────────────────────────── */

static void update_backlight(void) {
    static bool last_state = false;
    bool active = vario_data->settings.backlight_forced ||
                  (furi_get_tick() - vario_data->last_button_time < 5000u);
    if(active != last_state) {
        if(active) notification_message(notification, &sequence_display_backlight_on);
        last_state = active;
    }
}

/* ── Индикатор вертикальной скорости ─────────────────────────────────── */

/*
 * Вертикальная полоса слева экрана (VBAR_X, VBAR_W px).
 * Центр = нейтраль (VBAR_CY). Заливка и стрелка пропорциональны скорости.
 * Полная шкала: VBAR_MAX_MS м/с → VBAR_HALF пикселей.
 */
static void draw_vbar(Canvas* canvas, float vspeed) {
    int cx = VBAR_X + VBAR_W / 2;

    /* Рамка и центральная линия */
    canvas_draw_frame(canvas, VBAR_X, VBAR_CY - VBAR_HALF, VBAR_W, VBAR_HALF * 2);
    canvas_draw_line(canvas, VBAR_X, VBAR_CY, VBAR_X + VBAR_W - 1, VBAR_CY);

    float clamped = vspeed;
    if(clamped >  VBAR_MAX_MS) clamped =  VBAR_MAX_MS;
    if(clamped < -VBAR_MAX_MS) clamped = -VBAR_MAX_MS;
    int fill_px = (int)(clamped / VBAR_MAX_MS * (float)VBAR_HALF);

    if(fill_px > 1) {
        /* Подъём: заливка вверх, стрелка вверху */
        int top = VBAR_CY - fill_px;
        for(int y = top; y < VBAR_CY; y++)
            canvas_draw_line(canvas, VBAR_X + 1, y, VBAR_X + VBAR_W - 2, y);
        /* Треугольник острием вверх */
        for(int i = 0; i <= VBAR_W / 2; i++)
            canvas_draw_line(canvas,
                             cx - i, top - (VBAR_W / 2 - i),
                             cx + i, top - (VBAR_W / 2 - i));

    } else if(fill_px < -1) {
        /* Снижение: заливка вниз, стрелка внизу */
        int bot = VBAR_CY - fill_px;
        for(int y = VBAR_CY + 1; y <= bot; y++)
            canvas_draw_line(canvas, VBAR_X + 1, y, VBAR_X + VBAR_W - 2, y);
        /* Треугольник острием вниз */
        for(int i = 0; i <= VBAR_W / 2; i++)
            canvas_draw_line(canvas,
                             cx - i, bot + (VBAR_W / 2 - i),
                             cx + i, bot + (VBAR_W / 2 - i));
    }
}

/* ── Отрисовка ────────────────────────────────────────────────────────── */

static void draw_callback(Canvas* canvas, void* ctx) {
    UNUSED(ctx);

    if(!vario_data) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 30, "Initializing...");
        return;
    }

    furi_mutex_acquire(vario_data->mutex, FuriWaitForever);

    /* ── Меню настроек ────────────────────────────────────────────────── */
    if(vario_data->show_menu) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 2, 11, "Settings");
        canvas_draw_line(canvas, 0, 13, 127, 13);

        canvas_set_font(canvas, FontSecondary);

        /*
         * Рамка на всю ширину экрана (0..127) — не прыгает при смене пункта.
         * Координаты рассчитаны через макросы MENU_ROW_Y / MENU_FRAME_TOP.
         * Значение ON/OFF выровнено по правому краю на x=90.
         */

        /* Пункт 0: Sound */
        if(vario_data->menu_selection == 0)
            canvas_draw_frame(canvas, 0, MENU_FRAME_TOP(0), 128, MENU_FRAME_H);
        canvas_draw_str(canvas, 4, MENU_ROW_Y(0), "Sound:");
        canvas_draw_str(canvas, 90, MENU_ROW_Y(0),
                        vario_data->settings.sound_enabled ? "ON" : "OFF");

        /* Пункт 1: Backlight */
        if(vario_data->menu_selection == 1)
            canvas_draw_frame(canvas, 0, MENU_FRAME_TOP(1), 128, MENU_FRAME_H);
        canvas_draw_str(canvas, 4, MENU_ROW_Y(1), "Backlight:");
        canvas_draw_str(canvas, 90, MENU_ROW_Y(1),
                        vario_data->settings.backlight_forced ? "ON" : "OFF");

        /* Пункт 2: Logging */
        if(vario_data->menu_selection == 2)
            canvas_draw_frame(canvas, 0, MENU_FRAME_TOP(2), 128, MENU_FRAME_H);
        canvas_draw_str(canvas, 4, MENU_ROW_Y(2), "Logging:");
        canvas_draw_str(canvas, 90, MENU_ROW_Y(2),
                        vario_data->settings.logging_enabled ? "ON" : "OFF");

        /* Пункт 3: Log interval */
        if(vario_data->menu_selection == 3)
            canvas_draw_frame(canvas, 0, MENU_FRAME_TOP(3), 128, MENU_FRAME_H);
        canvas_draw_str(canvas, 4, MENU_ROW_Y(3), "Log interval:");
        char ival[10];
        snprintf(ival, sizeof(ival), "%us", (unsigned)vario_data->settings.log_interval);
        canvas_draw_str(canvas, 100, MENU_ROW_Y(3), ival);

        furi_mutex_release(vario_data->mutex);
        return;
    }

    /* ── Датчик не найден ─────────────────────────────────────────────── */
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

    /* ── Основной экран ───────────────────────────────────────────────── */
    float vspeed = vario_data->varioS * 0.01f;

    draw_vbar(canvas, vspeed);

    /* Левый край контентной зоны — после полосы и стрелки */
    int cx = VBAR_X + VBAR_W + VBAR_W / 2 + 3;

    char buf[32];

    /* Статусные иконки */
    canvas_set_font(canvas, FontSecondary);
    if(!vario_data->settings.sound_enabled)
        canvas_draw_str(canvas, cx, 8, "MUTE");
    if(vario_data->settings.logging_enabled)
        canvas_draw_str(canvas, 105, 8, "LOG");

    /* Высота */
    canvas_set_font(canvas, FontBigNumbers);
    snprintf(buf, sizeof(buf), "%.0f", (double)vario_data->altitude);
    canvas_draw_str_aligned(canvas, 70, 20, AlignCenter, AlignBottom, buf);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 70, 22, AlignCenter, AlignTop, "m");

    /* Вертикальная скорость */
    canvas_set_font(canvas, FontPrimary);
    if(fabsf(vspeed) < 0.01f)
        snprintf(buf, sizeof(buf), "0.00 m/s");
    else
        snprintf(buf, sizeof(buf), "%+.2f m/s", (double)vspeed);
    canvas_draw_str_aligned(canvas, 70, 38, AlignCenter, AlignTop, buf);

    /* Температура и давление */
    canvas_set_font(canvas, FontSecondary);
    snprintf(buf, sizeof(buf), "%.1fC", (double)vario_data->temperature);
    canvas_draw_str(canvas, cx, 56, buf);
    snprintf(buf, sizeof(buf), "%.0fhPa", (double)(vario_data->pressure / 100.0f));
    canvas_draw_str_aligned(canvas, 126, 56, AlignRight, AlignTop, buf);

    furi_mutex_release(vario_data->mutex);
}

/* ── Кнопки ───────────────────────────────────────────────────────────── */

static void handle_input(InputEvent* event, void* context) {
    FuriMessageQueue* queue = (FuriMessageQueue*)context;
    furi_message_queue_put(queue, event, 0);
}

/* ── Рабочий поток ────────────────────────────────────────────────────── */

static int32_t vario_worker(void* context) {
    UNUSED(context);

    /* Сбрасываем статические переменные звука на случай повторного запуска */
    sound_speaker_owned = false;
    sound_next_beep     = 0;
    sound_beep_end      = 0;

    vario_data->settings.sound_enabled    = true;
    vario_data->settings.backlight_forced = false;
    vario_data->settings.logging_enabled  = false;
    vario_data->settings.log_interval     = 10;
    vario_data->last_log_time             = 0;
    vario_data->last_button_time          = furi_get_tick();
    vario_data->menu_selection            = 0;
    vario_data->varioS                    = 0.0f;

    FURI_LOG_I(TAG, "Worker started, init BME280...");

    if(bme280_init_sensor()) {
        vario_data->sensor_ready = true;
        last_time_kf = (float)furi_get_tick() / 1000.0f;
        kf_reset(SEA_LEVEL_PRESSURE, 0.0f);
        FURI_LOG_I(TAG, "BME280 ready");
    } else {
        vario_data->sensor_ready = false;
        FURI_LOG_E(TAG, "BME280 not found, will retry");
    }

    uint32_t last_retry = 0;

    while(vario_data->running) {
        if(vario_data->sensor_ready) {
            update_pressure();
            sound_update();
            log_to_file();
        } else {
            uint32_t now = furi_get_tick();
            if(now - last_retry > 5000u) {
                last_retry = now;
                if(bme280_init_sensor()) {
                    vario_data->sensor_ready = true;
                    last_time_kf = (float)furi_get_tick() / 1000.0f;
                    kf_reset(SEA_LEVEL_PRESSURE, 0.0f);
                }
            }
        }
        furi_delay_ms(200);
    }

    /* Гарантированно останавливаем динамик перед выходом.
     * Используем acquire с таймаутом чтобы точно получить доступ. */
    if(sound_speaker_owned) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
        sound_speaker_owned = false;
    } else if(furi_hal_speaker_acquire(100)) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }

    FURI_LOG_I(TAG, "Worker stopped");
    return 0;
}

/* ── Точка входа ──────────────────────────────────────────────────────── */

int32_t vario_app(void* p) {
    UNUSED(p);

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    vario_data = malloc(sizeof(VarioData));
    memset(vario_data, 0, sizeof(VarioData));
    vario_data->mutex   = furi_mutex_alloc(FuriMutexTypeNormal);
    vario_data->running = true;

    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, NULL);
    view_port_input_callback_set(view_port, handle_input, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);
    notification = furi_record_open(RECORD_NOTIFICATION);

    FuriThread* worker = furi_thread_alloc();
    furi_thread_set_name(worker, "VarioWorker");
    furi_thread_set_stack_size(worker, 4096);
    furi_thread_set_context(worker, vario_data);
    furi_thread_set_callback(worker, vario_worker);
    furi_thread_start(worker);

    InputEvent event;
    while(vario_data->running) {
        if(furi_message_queue_get(event_queue, &event, 100) == FuriStatusOk) {
            furi_mutex_acquire(vario_data->mutex, FuriWaitForever);
            vario_data->last_button_time = furi_get_tick();

            /* Back вне меню = выход */
            if(event.type == InputTypeShort && event.key == InputKeyBack
               && !vario_data->show_menu) {
                vario_data->running = false;
                furi_mutex_release(vario_data->mutex);
                break;
            }

            if(event.type == InputTypeShort) {
                if(vario_data->show_menu) {
                    switch(event.key) {
                    case InputKeyUp:
                        vario_data->menu_selection = (vario_data->menu_selection + 3u) % 4u;
                        break;
                    case InputKeyDown:
                        vario_data->menu_selection = (vario_data->menu_selection + 1u) % 4u;
                        break;
                    case InputKeyOk:
                        switch(vario_data->menu_selection) {
                        case 0:
                            vario_data->settings.sound_enabled =
                                !vario_data->settings.sound_enabled;
                            break;
                        case 1:
                            vario_data->settings.backlight_forced =
                                !vario_data->settings.backlight_forced;
                            break;
                        case 2:
                            vario_data->settings.logging_enabled =
                                !vario_data->settings.logging_enabled;
                            break;
                        case 3:
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
                        /* Back в меню = закрыть меню, не выходить */
                        vario_data->show_menu = false;
                        break;
                    default:
                        break;
                    }
                } else {
                    switch(event.key) {
                    case InputKeyUp:
                        vario_data->settings.sound_enabled =
                            !vario_data->settings.sound_enabled;
                        break;
                    case InputKeyDown:
                        vario_data->settings.backlight_forced =
                            !vario_data->settings.backlight_forced;
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

    /* ── Завершение ───────────────────────────────────────────────────── */
    vario_data->running = false;

    /* Ждём воркера — он сам остановит динамик в конце своего цикла */
    furi_thread_join(worker);
    furi_thread_free(worker);

    view_port_enabled_set(view_port, false);
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    furi_mutex_free(vario_data->mutex);
    if(vario_data->bme280) free(vario_data->bme280);
    free(vario_data);
    vario_data = NULL;

    furi_message_queue_free(event_queue);
    return 0;
}
