#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <linux/rtc.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h> /* I2C 처리를 위한 Workqueue */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("User");
MODULE_DESCRIPTION("Integrated DS1302 RTC & OLED Display Driver");

/* ========================================================================== */
/* 1. 핀 및 상수 정의                          */
/* ========================================================================== */

// --- DS1302 & Encoder Pins ---
#define DS1302_CLK  14
#define DS1302_IO   15
#define DS1302_RST  18 

#define ROT_S1      23
#define ROT_S2      24
#define ROT_KEY     25

// --- OLED I2C Settings ---
#define DRIVER_NAME "dol12864_combined"
#define OLED_I2C_ADDR 0x3C
#define I2C_BUS_NUMBER 1

// --- UI Constants ---
#define STATE_NAV   0  // 탐색 모드
#define STATE_EDIT  1  // 수정 모드

enum {
    FIELD_YEAR = 0,
    FIELD_MONTH,
    FIELD_DATE,
    FIELD_HOUR,
    FIELD_MIN,
    FIELD_MAX
};

/* ========================================================================== */
/* 2. 전역 변수 및 폰트                        */
/* ========================================================================== */

// --- OLED Font Data ---
static const unsigned char big_font[][16] = {
    // 0
    {0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00, 0x00},
    // 1
    {0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, 0x00},
    // 2
    {0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00, 0x00},
    // 3
    {0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00, 0x00, 0x18, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 4
    {0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x07, 0x04, 0x24, 0x24, 0x3F, 0x24, 0x00, 0x00},
    // 5
    {0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00, 0x19, 0x21, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 6
    {0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00, 0x00, 0x0F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 7
    {0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00},
    // 8
    {0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00, 0x00},
    // 9
    {0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x00, 0x31, 0x22, 0x22, 0x11, 0x0F, 0x00, 0x00},
    // : (Index 10)
    {0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00},
    // - (Index 11)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00},
};

static const unsigned char oled_init_cmds[] = {
    0x00, 0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF, 0xA1, 0xA6, 
    0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20, 
    0x8D, 0x14, 0xAF
};

// --- DS1302 Data Structures ---
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

// --- Global Variables ---
static t_ds1302 ds_time;
static int app_state = STATE_NAV;
static int current_focus = FIELD_YEAR;

static struct i2c_client *oled_client_device = NULL;
static struct timer_list my_timer;
static int irq_num_s1;
static int irq_num_key;
static unsigned long last_rotary_time = 0;
static unsigned long last_key_time = 0;

// 인터럽트에서 I2C를 호출하기 위한 Workqueue
static struct work_struct oled_update_work;

/* ========================================================================== */
/* 3. DS1302 Low Level Functions                     */
/* ========================================================================== */

uint8_t bcd2dec(uint8_t byte) { return ((byte >> 4) * 10) + (byte & 0x0f); }
uint8_t dec2bcd(uint8_t byte) { return ((byte / 10) << 4) + (byte % 10); }

void ds1302_clk_pulse(void) {
    gpio_set_value(DS1302_CLK, 1); udelay(2);
    gpio_set_value(DS1302_CLK, 0); udelay(2);
}

void ds1302_tx(uint8_t tx) {
    int i;
    gpio_direction_output(DS1302_IO, 0);
    for (i = 0; i < 8; i++) {
        gpio_set_value(DS1302_IO, (tx & (1 << i)) ? 1 : 0);
        udelay(1); ds1302_clk_pulse();
    }
}

void ds1302_rx(uint8_t *data) {
    int i; uint8_t temp = 0;
    gpio_direction_input(DS1302_IO);
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

void update_ds1302_value(uint8_t target_addr, int direction, int min_val, int max_val) {
    uint8_t current_val = ds1302_read(target_addr);
    int new_val = (int)current_val;

    if (direction > 0) { 
        new_val++;
        if (new_val > max_val) new_val = min_val;
    } else { 
        new_val--;
        if (new_val < min_val) new_val = max_val;
    }

    ds1302_write_raw(ADDR_WP, 0x00); // WP 해제
    ds1302_write(target_addr, (uint8_t)new_val);
}

/* ========================================================================== */
/* 4. OLED Display Logic                            */
/* ========================================================================== */

static void oled_set_cursor(struct i2c_client *client, int page, int col) {
    unsigned char cmds[] = {
        0x00, 0xB0 + page, 0x00 + (col & 0x0F), 0x10 + ((col >> 4) & 0x0F)
    };
    i2c_master_send(client, cmds, sizeof(cmds));
}

static void oled_print_big_char(struct i2c_client *client, int page, int col, int char_idx, int invert) {
    int i;
    unsigned char top_buf[9], bot_buf[9], d;
    top_buf[0] = 0x40; bot_buf[0] = 0x40;
    
    for(i=0; i<8; i++) {
        d = big_font[char_idx][i];
        if(invert) d = ~d;
        top_buf[i+1] = d;

        d = big_font[char_idx][i+8];
        if(invert) d = ~d;
        bot_buf[i+1] = d;
    }
    oled_set_cursor(client, page, col);
    i2c_master_send(client, top_buf, 9);
    oled_set_cursor(client, page + 1, col);
    i2c_master_send(client, bot_buf, 9);
}

static void oled_clear(struct i2c_client *client) {
    int page;
    unsigned char buf[129];
    buf[0] = 0x40;
    memset(buf + 1, 0x00, 128);
    for(page = 0; page < 8; page++) {
        oled_set_cursor(client, page, 0);
        i2c_master_send(client, buf, 129);
    }
}

// 화면 업데이트 함수 (Workqueue에서 호출됨)
static void oled_refresh_screen(struct i2c_client *client) {
    char time_str[20];
    int i, char_code, invert;
    
    // ds_time은 타이머나 로터리 ISR에서 최신화되어 있음
    // 포맷: "YY-MM-DDHH:MM:SS" (총 16글자 매핑)
    snprintf(time_str, sizeof(time_str), "%02d-%02d-%02d%02d:%02d:%02d", 
             ds_time.year, ds_time.month, ds_time.date,
             ds_time.hours, ds_time.minutes, ds_time.seconds);
             
    // 윗줄: 날짜 (0~7) "YY-MM-DD"
    for(i=0; i<8; i++) {
        char c = time_str[i];
        if(c >= '0' && c <= '9') char_code = c - '0';
        else char_code = 11; // '-'
        
        invert = 0;
        // NAV 모드이거나 EDIT 모드일 때 포커스 된 항목 반전 표시
        if (current_focus == FIELD_YEAR && (i==0 || i==1)) invert = 1;
        if (current_focus == FIELD_MONTH && (i==3 || i==4)) invert = 1;
        if (current_focus == FIELD_DATE && (i==6 || i==7)) invert = 1;

        oled_print_big_char(client, 0, 16 + (i * 12), char_code, invert);
    }

    // 아랫줄: 시간 (8~15) "HH:MM:SS"
    for(i=8; i<16; i++) { 
        char c = time_str[i];
        int x_pos = 8 + ((i-8) * 14); 

        if(c >= '0' && c <= '9') char_code = c - '0';
        else char_code = 10; // ':'
        
        invert = 0;
        if (current_focus == FIELD_HOUR && (i==8 || i==9))   invert = 1;
        if (current_focus == FIELD_MIN && (i==11 || i==12)) invert = 1;
        // 초(SS)는 수정 대상이 아니므로 반전 없음
        
        oled_print_big_char(client, 3, x_pos, char_code, invert);
    }
}

// Workqueue Handler: ISR에서 예약된 작업을 처리 (여기서는 I2C 전송이 안전함)
static void update_display_worker(struct work_struct *work) {
    if (oled_client_device) {
        oled_refresh_screen(oled_client_device);
    }
}

/* ========================================================================== */
/* 5. Interrupts & Timer                              */
/* ========================================================================== */

static irqreturn_t rotary_isr(int irq, void *dev_id) {
    unsigned long current_time = jiffies;
    int s1_val, s2_val, direction = 0;

    if (time_before(current_time, last_rotary_time + msecs_to_jiffies(100))) 
        return IRQ_HANDLED;
    last_rotary_time = current_time;

    s1_val = gpio_get_value(ROT_S1);
    s2_val = gpio_get_value(ROT_S2);

    if (s1_val == 0) {
        if (s2_val == 1) direction = 1; else direction = -1;

        if (app_state == STATE_NAV) {
            // [탐색] 항목 이동
            if (direction > 0) {
                current_focus++;
                if (current_focus >= FIELD_MAX) current_focus = 0;
            } else {
                current_focus--;
                if (current_focus < 0) current_focus = FIELD_MAX - 1;
            }
        } 
        else if (app_state == STATE_EDIT) {
            // [수정] DS1302 값 변경
            switch (current_focus) {
                case FIELD_YEAR:  update_ds1302_value(ADDR_YEAR, direction, 0, 99); break;
                case FIELD_MONTH: update_ds1302_value(ADDR_MONTH, direction, 1, 12); break;
                case FIELD_DATE:  update_ds1302_value(ADDR_DATE, direction, 1, 31); break;
                case FIELD_HOUR:  update_ds1302_value(ADDR_HOURS, direction, 0, 23); break;
                case FIELD_MIN:   
                    update_ds1302_value(ADDR_MINUTES, direction, 0, 59);
                    ds1302_write(ADDR_SECONDS, 0); // 초 초기화
                    break;
            }
            // 변경 즉시 값을 전역변수로 읽어와서 화면에 반영되게 함
            ds1302_read_all();
        }
        // 화면 갱신 예약
        schedule_work(&oled_update_work);
    }
    return IRQ_HANDLED;
}

static irqreturn_t key_isr(int irq, void *dev_id) {
    unsigned long current_time = jiffies;

    if (time_before(current_time, last_key_time + msecs_to_jiffies(300)))
        return IRQ_HANDLED;
    last_key_time = current_time;

    // 상태 토글 (NAV <-> EDIT)
    if (app_state == STATE_NAV) {
        app_state = STATE_EDIT;
    } else {
        app_state = STATE_NAV;
    }
    
    // 상태 변경 즉시 화면 반영
    schedule_work(&oled_update_work);
    return IRQ_HANDLED;
}

void timer_callback(struct timer_list *timer) {
    // 주기적으로 시간 읽기
    ds1302_read_all();
    
    // 화면 갱신 예약 (직접 호출하지 않고 Workqueue 사용)
    schedule_work(&oled_update_work);

    mod_timer(&my_timer, jiffies + msecs_to_jiffies(1000));
}

/* ========================================================================== */
/* 6. Driver Init / Exit                             */
/* ========================================================================== */

static int oled_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    i2c_master_send(client, oled_init_cmds, sizeof(oled_init_cmds));
    oled_clear(client);
    pr_info("%s: OLED Initialized\n", DRIVER_NAME);
    return 0;
}

static void oled_remove(struct i2c_client *client) {
    unsigned char off_cmd[] = {0x00, 0xAE};
    i2c_master_send(client, off_cmd, sizeof(off_cmd));
}

static const struct i2c_device_id oled_id[] = { { "dol12864", 0 }, { } };
MODULE_DEVICE_TABLE(i2c, oled_id);

static struct i2c_driver oled_driver = {
    .driver = { .name = DRIVER_NAME, .owner = THIS_MODULE },
    .probe = oled_probe,
    .remove = oled_remove,
    .id_table = oled_id,
};

static int __init combined_driver_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info;
    int ret;

    pr_info("%s: Combined Driver Loading...\n", DRIVER_NAME);

    // 1. GPIO Init (DS1302 & Encoder)
    gpio_request(DS1302_CLK, "DS1302_CLK"); gpio_direction_output(DS1302_CLK, 0);
    gpio_request(DS1302_IO,  "DS1302_IO");  gpio_direction_output(DS1302_IO, 0);
    gpio_request(DS1302_RST, "DS1302_RST"); gpio_direction_output(DS1302_RST, 0);
    gpio_request(ROT_S1, "ROT_S1"); gpio_direction_input(ROT_S1);
    gpio_request(ROT_S2, "ROT_S2"); gpio_direction_input(ROT_S2);
    gpio_request(ROT_KEY, "ROT_KEY"); gpio_direction_input(ROT_KEY);

    irq_num_s1 = gpio_to_irq(ROT_S1);
    ret = request_irq(irq_num_s1, rotary_isr, IRQF_TRIGGER_FALLING, "rotary_irq", NULL);
    if (ret) { pr_err("Failed to request IRQ S1\n"); return ret; }

    irq_num_key = gpio_to_irq(ROT_KEY);
    ret = request_irq(irq_num_key, key_isr, IRQF_TRIGGER_FALLING, "key_irq", NULL);
    if (ret) { free_irq(irq_num_s1, NULL); return ret; }

    // 2. Workqueue Init
    INIT_WORK(&oled_update_work, update_display_worker);

    // 3. I2C Driver & Device Init
    ret = i2c_add_driver(&oled_driver);
    if (ret < 0) return ret;

    adapter = i2c_get_adapter(I2C_BUS_NUMBER);
    if (!adapter) {
        i2c_del_driver(&oled_driver);
        return -ENODEV;
    }

    memset(&board_info, 0, sizeof(struct i2c_board_info));
    strscpy(board_info.type, "dol12864", I2C_NAME_SIZE);
    board_info.addr = OLED_I2C_ADDR;

    oled_client_device = i2c_new_client_device(adapter, &board_info);
    i2c_put_adapter(adapter);

    if (IS_ERR(oled_client_device)) {
        i2c_del_driver(&oled_driver);
        return PTR_ERR(oled_client_device);
    }

    // 4. Timer Init (Start Ticking)
    ds1302_read_all(); // 초기값 읽기
    timer_setup(&my_timer, timer_callback, 0);
    mod_timer(&my_timer, jiffies + msecs_to_jiffies(1000));

    pr_info("%s: Setup Complete.\n", DRIVER_NAME);
    return 0;
}

static void __exit combined_driver_exit(void)
{
    // 순서: 타이머 중지 -> Workqueue 취소 -> 장치 제거
    del_timer_sync(&my_timer);
    cancel_work_sync(&oled_update_work);

    if (oled_client_device) i2c_unregister_device(oled_client_device);
    i2c_del_driver(&oled_driver);

    free_irq(irq_num_s1, NULL);
    free_irq(irq_num_key, NULL);
    
    gpio_free(DS1302_CLK); gpio_free(DS1302_IO); gpio_free(DS1302_RST);
    gpio_free(ROT_S1);     gpio_free(ROT_S2);    gpio_free(ROT_KEY);

    pr_info("%s: Driver Unloaded\n", DRIVER_NAME);
}

module_init(combined_driver_init);
module_exit(combined_driver_exit);