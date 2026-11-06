#include "night.h"

#include <fcntl.h>
#include <limits.h>
#include <string.h>

char nightOn = 0;
static bool grayscale = false, ircut = true, irled = false, manual = false;
pthread_t nightPid = 0;

bool night_grayscale_on(void) { return grayscale; }

bool night_ircut_on(void) { return ircut; }

bool night_irled_on(void) { return irled; }

bool night_manual_on(void) { return manual; }

bool night_mode_on(void) { return grayscale && !ircut && irled; }

void night_grayscale(bool enable) {
    set_grayscale(enable);
    grayscale = enable;
}

void night_ircut(bool enable) {
    bool has1 = app_config.ir_cut_pin1 != 999;
    bool has2 = app_config.ir_cut_pin2 != 999;

    if (has1) gpio_write(app_config.ir_cut_pin1, !enable);
    if (has2) gpio_write(app_config.ir_cut_pin2, enable);
    if (has1 || has2) {
        usleep(app_config.pin_switch_delay_us * 100);
        if (has1) gpio_write(app_config.ir_cut_pin1, false);
        if (has2) gpio_write(app_config.ir_cut_pin2, false);
    }
    ircut = enable;
}

void night_irled(bool enable) {
    if (app_config.ir_led_pin != 999)
        gpio_write(app_config.ir_led_pin, enable);
    irled = enable;
}

void night_manual(bool enable) { manual = enable; }

void night_mode(bool enable) {
    HAL_INFO("night", "Changing mode to %s\n", enable ? "NIGHT" : "DAY");
#if defined(__arm__) && !defined(__ARM_PCS_VFP) && __ARM_ARCH == 6
    if (plat == HAL_PLATFORM_F1) {
        hal_light lamp = app_config.night_lamp;
        if (!enable) {
            night_grayscale(false);
            night_ircut(true);
            night_irled(false);
            fh_lamp_enable(HAL_LIGHT_NONE);
        } else if (lamp & HAL_LIGHT_WHITE) {
            night_grayscale(false);
            night_ircut(true);
            night_irled((lamp & HAL_LIGHT_IR) != 0);
            fh_lamp_enable(lamp);
        } else {
            night_grayscale(true);
            night_ircut(false);
            night_irled((lamp & HAL_LIGHT_IR) != 0);
            fh_lamp_enable(lamp);
        }
        return;
    }
#endif
    night_grayscale(enable);
    night_ircut(!enable);
    night_irled(enable);
}

static bool adc_read(int fd, int *val) {
    if (app_config.adc_channel < 0)
        return read(fd, val, sizeof(*val)) > 0;

    char buf[512], key[16];
    ssize_t n;
    if (lseek(fd, 0, SEEK_SET) < 0 || (n = read(fd, buf, sizeof(buf) - 1)) <= 0)
        return false;
    buf[n] = 0;
    snprintf(key, sizeof(key), "adc%d=", app_config.adc_channel);
    for (char *p = buf; (p = strstr(p, key)); p++)
        if (p == buf || p[-1] == '\n' || p[-1] == ' ') {
            *val = (int)strtol(p + strlen(key), NULL, 0);
            return true;
        }
    return false;
}

void *night_thread(void) {
    gpio_init();
    usleep(10000);

    night_mode(night_mode_on());

#if defined(__arm__) && !defined(__ARM_PCS_VFP) && __ARM_ARCH == 6
    if (plat == HAL_PLATFORM_F1 && f1_night_available() &&
        !app_config.adc_device[0] && app_config.ir_sensor_pin == 999) {
        HAL_INFO("night", "Using image gain for day/night switching\n");
        int applied = -1;
        while (keepRunning && nightOn) {
            int state = f1_night_status();
            if (manual) applied = -1;
            else if (state != applied) { night_mode(state); applied = state; }
            usleep(40000);
        }
    } else
#endif
    if (app_config.adc_device[0]) {
        int adc_fd = -1;
        int cnt = 0, tmp = 0, val, state = manual ? -1 : night_mode_on();
        int thr_n = app_config.adc_threshold, thr_d = app_config.adc_threshold_day;
        bool low_dark = thr_d != INT_MIN && thr_d > thr_n;
        if (thr_d == INT_MIN) thr_d = thr_n;

        if ((adc_fd = open(app_config.adc_device, O_RDONLY | O_NONBLOCK)) < 0) {
            HAL_DANGER("night", "Could not open the ADC virtual device!\n");
            return NULL;
        }
        while (keepRunning && nightOn) {
            if (adc_read(adc_fd, &val)) {
                usleep(10000);
                tmp += val;
                cnt++;
            }
            if (cnt == 12) {
                tmp /= cnt;
                int next = state;
                if (low_dark) {
                    if (tmp < thr_n) next = 1;
                    else if (tmp > thr_d) next = 0;
                } else {
                    if (tmp >= thr_n) next = 1;
                    else if (tmp <= thr_d) next = 0;
                }
                if (next < 0) next = 0;
                if (!manual && next != state) night_mode(next);
                if (!manual) state = next;
                else state = -1;
                cnt = tmp = 0;
            }
            usleep(app_config.check_interval_s * 1000000 / 12);
        }
        close(adc_fd);
    } else if (app_config.ir_sensor_pin == 999) {
        while (keepRunning && nightOn) sleep(1);
    } else {
        while (keepRunning) {
            bool state = false;
            if (!gpio_read(app_config.ir_sensor_pin, &state))
                if (!manual) night_mode(state);

            sleep(app_config.check_interval_s);
        }
    }

    usleep(10000);
    gpio_deinit();
    HAL_INFO("night", "Night mode thread is closing...\n");
    nightOn = 0;
}

int night_enable(void) {
    int ret = EXIT_SUCCESS;

    if (nightOn) return ret;

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    size_t stacksize;
    pthread_attr_getstacksize(&thread_attr, &stacksize);
    size_t new_stacksize = 16 * 1024;
    if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", new_stacksize);
    pthread_create(&nightPid, &thread_attr, (void *(*)(void *))night_thread, NULL);
    if (pthread_attr_setstacksize(&thread_attr, stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", stacksize);
    pthread_attr_destroy(&thread_attr);

    nightOn = 1;

    return ret;
}

void night_disable(void) {
    if (!nightOn) return;

    nightOn = 0;
    pthread_join(nightPid, NULL);
}
