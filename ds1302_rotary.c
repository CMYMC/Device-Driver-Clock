#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("User");
MODULE_DESCRIPTION("DS1302 Nav/Edit Mode Driver");

/* --- [1] 핀 설정 --- */
#define DS1302_CLK  14
#define DS1302_IO   15
#define DS1302_RST  18 

#define ROT_S1      23
#define ROT_S2      24
#define ROT_KEY     25

/* --- [2] 상태 및 항목 정의 --- */
// 시스템 상태
#define STATE_NAV   0  // 탐색 모드 (항목 선택)
#define STATE_EDIT  1  // 수정 모드 (값 변경)

// 수정할 항목 인덱스
enum {
    FIELD_YEAR = 0,
    FIELD_MONTH,
    FIELD_DATE,
    FIELD_HOUR,
    FIELD_MIN,
    FIELD_MAX
};

char *field_names[] = {"YEAR", "MONTH", "DATE", "HOUR", "MINUTE"};

// 전역 변수
// ★ 수정: system_state -> app_state (이름 충돌 방지) ★
static int app_state = STATE_NAV;      // 현재 상태 (기본: 탐색)
static int current_focus = FIELD_YEAR; // 현재 선택된 항목 (기본: 년도)

/* --- [3] DS1302 관련 정의 --- */
#define ADDR_SECONDS    0x80
#define ADDR_MINUTES    0x82
#define ADDR_HOURS      0x84
#define ADDR_DATE       0x86
#define ADDR_MONTH      0x88
#define ADDR_DAYOFWEEK  0x8a
#define ADDR_YEAR       0x8C
#define ADDR_WP         0x8E

typedef struct {
    uint8_t seconds; uint8_t minutes; uint8_t hours;
    uint8_t date;    uint8_t month;   uint8_t dayofweek; uint8_t year;
} t_ds1302;

t_ds1302 ds_time;
static struct timer_list my_timer;
static int irq_num_s1;
static int irq_num_key;
static unsigned long last_rotary_time = 0;
static unsigned long last_key_time = 0;

/* --- [4] DS1302 Low Level 함수 --- */
uint8_t bcd2dec(uint8_t byte) { return ((byte >> 4) * 10) + (byte & 0x0f); }
uint8_t dec2bcd(uint8_t byte) { return ((byte / 10) << 4) + (byte % 10); }

void ds1302_io_out(void) { gpio_direction_output(DS1302_IO, 0); }
void ds1302_io_in(void)  { gpio_direction_input(DS1302_IO); }

void ds1302_clk_pulse(void) {
    gpio_set_value(DS1302_CLK, 1); udelay(2);
    gpio_set_value(DS1302_CLK, 0); udelay(2);
}

void ds1302_tx(uint8_t tx) {
    int i;
    ds1302_io_out();
    for (i = 0; i < 8; i++) {
        gpio_set_value(DS1302_IO, (tx & (1 << i)) ? 1 : 0);
        udelay(1); ds1302_clk_pulse();
    }
}

void ds1302_rx(uint8_t *data) {
    int i; uint8_t temp = 0;
    ds1302_io_in();
    for (i = 0; i < 8; i++) {
        if (gpio_get_value(DS1302_IO)) temp |= (1 << i);
        if (i != 7) ds1302_clk_pulse();
    }
    *data = temp;
}

uint8_t ds1302_read(uint8_t addr) {
    uint8_t data;
    gpio_set_value(DS1302_RST, 1); udelay(4);
    ds1302_tx(addr + 1);
    ds1302_rx(&data);
    gpio_set_value(DS1302_RST, 0); udelay(4);
    return bcd2dec(data);
}

void ds1302_write(uint8_t addr, uint8_t val) {
    gpio_set_value(DS1302_RST, 1); udelay(4);
    ds1302_tx(addr);
    ds1302_tx(dec2bcd(val));
    gpio_set_value(DS1302_RST, 0); udelay(4);
}

void ds1302_write_raw(uint8_t addr, uint8_t val) {
    gpio_set_value(DS1302_RST, 1); udelay(4);
    ds1302_tx(addr);
    ds1302_tx(val); 
    gpio_set_value(DS1302_RST, 0); udelay(4);
}

void ds1302_read_all(void) {
    ds_time.seconds = ds1302_read(ADDR_SECONDS);
    ds_time.minutes = ds1302_read(ADDR_MINUTES);
    ds_time.hours   = ds1302_read(ADDR_HOURS);
    ds_time.date    = ds1302_read(ADDR_DATE);
    ds_time.month   = ds1302_read(ADDR_MONTH);
    ds_time.dayofweek = ds1302_read(ADDR_DAYOFWEEK);
    ds_time.year    = ds1302_read(ADDR_YEAR);
}

/* --- [5] 값 변경 로직 (수정 모드에서 사용) --- */
void update_ds1302_value(uint8_t target_addr, int direction, int min_val, int max_val) {
    uint8_t current_val = ds1302_read(target_addr);
    int new_val = (int)current_val;

    if (direction > 0) { // 증가
        new_val++;
        if (new_val > max_val) new_val = min_val;
    } else { // 감소
        new_val--;
        if (new_val < min_val) new_val = max_val;
    }

    ds1302_write_raw(ADDR_WP, 0x00); // WP 해제
    ds1302_write(target_addr, (uint8_t)new_val);
}

/* --- [6] 인터럽트 핸들러 (엔코더 회전) --- */
static irqreturn_t rotary_isr(int irq, void *dev_id) {
    unsigned long current_time = jiffies;
    int s1_val, s2_val;
    int direction = 0;

    if (time_before(current_time, last_rotary_time + msecs_to_jiffies(100))) {
        return IRQ_HANDLED;
    }
    last_rotary_time = current_time;

    s1_val = gpio_get_value(ROT_S1);
    s2_val = gpio_get_value(ROT_S2);

    if (s1_val == 0) {
        // 1. 방향 판별
        if (s2_val == 1) direction = 1;  // CW (시계)
        else direction = -1; // CCW (반시계)

        // 2. 상태에 따른 분기 처리 (app_state 사용)
        if (app_state == STATE_NAV) {
            // [탐색 모드] : 포커스(항목) 이동
            if (direction > 0) {
                current_focus++;
                if (current_focus >= FIELD_MAX) current_focus = 0;
            } else {
                current_focus--;
                if (current_focus < 0) current_focus = FIELD_MAX - 1;
            }
            printk(KERN_INFO "[NAV] Selected Field: %s\n", field_names[current_focus]);
        } 
        else if (app_state == STATE_EDIT) {
            // [수정 모드] : 선택된 항목의 값 변경
            switch (current_focus) {
                case FIELD_YEAR:
                    update_ds1302_value(ADDR_YEAR, direction, 0, 99);
                    printk(KERN_INFO "[EDIT] Year Changing...\n");
                    break;
                case FIELD_MONTH:
                    update_ds1302_value(ADDR_MONTH, direction, 1, 12);
                    printk(KERN_INFO "[EDIT] Month Changing...\n");
                    break;
                case FIELD_DATE:
                    update_ds1302_value(ADDR_DATE, direction, 1, 31);
                    printk(KERN_INFO "[EDIT] Date Changing...\n");
                    break;
                case FIELD_HOUR:
                    update_ds1302_value(ADDR_HOURS, direction, 0, 23);
                    printk(KERN_INFO "[EDIT] Hour Changing...\n");
                    break;
                case FIELD_MIN:
                    update_ds1302_value(ADDR_MINUTES, direction, 0, 59);
                    ds1302_write(ADDR_SECONDS, 0); // 분 변경시 초 초기화
                    printk(KERN_INFO "[EDIT] Minute Changing...\n");
                    break;
            }
        }
    }
    return IRQ_HANDLED;
}

/* --- [7] 인터럽트 핸들러 (버튼 클릭) --- */
static irqreturn_t key_isr(int irq, void *dev_id) {
    unsigned long current_time = jiffies;

    if (time_before(current_time, last_key_time + msecs_to_jiffies(300))) {
        return IRQ_HANDLED;
    }
    last_key_time = current_time;

    // 상태 토글 (NAV <-> EDIT)
    if (app_state == STATE_NAV) {
        app_state = STATE_EDIT;
        printk(KERN_INFO ">>> Enter EDIT MODE for [%s] <<<\n", field_names[current_focus]);
    } else {
        app_state = STATE_NAV;
        printk(KERN_INFO ">>> Exit to NAV MODE (Current: %s) <<<\n", field_names[current_focus]);
    }

    return IRQ_HANDLED;
}

/* --- [8] 타이머 핸들러 --- */
void timer_callback(struct timer_list *timer) {
    char *mode_str;
    char highlight_l = ' '; 
    char highlight_r = ' ';

    ds1302_read_all();

    if (app_state == STATE_NAV) {
        mode_str = "NAV";
        highlight_l = '['; highlight_r = ']'; // 탐색 중임을 표시
    } else {
        mode_str = "EDIT";
        highlight_l = '*'; highlight_r = '*'; // 수정 중임을 표시
    }

    // 현재 선택된 항목에 따라 로그 가독성을 높임
    printk(KERN_INFO "20%02d-%02d-%02d %02d:%02d:%02d | Mode: %s | Focus: %c%s%c\n",
           ds_time.year, ds_time.month, ds_time.date,
           ds_time.hours, ds_time.minutes, ds_time.seconds,
           mode_str, highlight_l, field_names[current_focus], highlight_r);

    mod_timer(&my_timer, jiffies + msecs_to_jiffies(1000));
}

/* --- [9] 초기화 및 종료 --- */
static int __init my_init(void) {
    int ret;
    printk(KERN_INFO "DS1302 Nav/Edit Driver Loading...\n");

    gpio_request(DS1302_CLK, "DS1302_CLK"); gpio_direction_output(DS1302_CLK, 0);
    gpio_request(DS1302_IO,  "DS1302_IO");  gpio_direction_output(DS1302_IO, 0);
    gpio_request(DS1302_RST, "DS1302_RST"); gpio_direction_output(DS1302_RST, 0);

    gpio_request(ROT_S1, "ROT_S1"); gpio_direction_input(ROT_S1);
    gpio_request(ROT_S2, "ROT_S2"); gpio_direction_input(ROT_S2);
    gpio_request(ROT_KEY, "ROT_KEY"); gpio_direction_input(ROT_KEY);

    irq_num_s1 = gpio_to_irq(ROT_S1);
    ret = request_irq(irq_num_s1, rotary_isr, IRQF_TRIGGER_FALLING, "rotary_irq", NULL);
    if (ret) return ret;

    irq_num_key = gpio_to_irq(ROT_KEY);
    ret = request_irq(irq_num_key, key_isr, IRQF_TRIGGER_FALLING, "key_irq", NULL);
    if (ret) { free_irq(irq_num_s1, NULL); return ret; }

    timer_setup(&my_timer, timer_callback, 0);
    mod_timer(&my_timer, jiffies + msecs_to_jiffies(1000));

    return 0;
}

static void __exit my_exit(void) {
    del_timer(&my_timer);
    free_irq(irq_num_s1, NULL);
    free_irq(irq_num_key, NULL);
    gpio_free(DS1302_CLK); gpio_free(DS1302_IO); gpio_free(DS1302_RST);
    gpio_free(ROT_S1);     gpio_free(ROT_S2);    gpio_free(ROT_KEY);
    printk(KERN_INFO "Driver Unloaded.\n");
}

module_init(my_init);
module_exit(my_exit);
