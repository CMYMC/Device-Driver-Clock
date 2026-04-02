#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <linux/rtc.h>

#define DRIVER_NAME "dol12864_oled"
#define OLED_I2C_ADDR 0x3C
#define I2C_BUS_NUMBER 1  // RPi4의 기본 I2C 버스 번호

static const unsigned char font[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (인덱스 0)
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H (인덱스 1)
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E (인덱스 2)
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L (인덱스 3)
    {0x3E, 0x41, 0x41, 0x41, 0x3E}  // O (인덱스 4)
};

static const unsigned char big_font[][16] = {
    // 0
    {0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00,  
     0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00, 0x00},
    // 1
    {0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,  
     0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, 0x00},
    // 2
    {0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00,  
     0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00, 0x00},
    // 3
    {0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00, 0x00,  
     0x18, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 4
    {0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00, 0x00,  
     0x07, 0x04, 0x24, 0x24, 0x3F, 0x24, 0x00, 0x00},
    // 5
    {0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00,  
     0x19, 0x21, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 6
    {0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00, 0x00,  
     0x0F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00},
    // 7
    {0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x00,  
     0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00},
    // 8
    {0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00,  
     0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00, 0x00},
    // 9
    {0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00,  
     0x00, 0x31, 0x22, 0x22, 0x11, 0x0F, 0x00, 0x00},
    // : (콜론, 인덱스 10)
    {0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00,  
     0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00},
    // - (하이픈, 인덱스 11)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
     0x00, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00},
};

/* SSD1306 초기화 커맨드 정의 */
static const unsigned char oled_init_cmds[] = {
    0x00, // Command Stream 모드
    0xAE, // Display OFF
    0x20, 0x00, // Set Memory Addressing Mode (Horizontal)
    0xB0, // Set Page Start Address for Page Addressing Mode
    0xC8, // Set COM Output Scan Direction
    0x00, // Set Low Column Address
    0x10, // Set High Column Address
    0x40, // Set Start Line Address
    0x81, 0xFF, // Set Contrast Control
    0xA1, // Set Segment Re-map
    0xA6, // Set Normal Display
    0xA8, 0x3F, // Set Multiplex Ratio (1 to 64)
    0xA4, // Entire Display ON (Resume)
    0xD3, 0x00, // Set Display Offset
    0xD5, 0xF0, // Set Display Clock Divide Ratio/Oscillator Frequency
    0xD9, 0x22, // Set Pre-charge Period
    0xDA, 0x12, // Set COM Pins Hardware Configuration
    0xDB, 0x20, // Set VCOMH Deselect Level
    0x8D, 0x14, // Charge Pump Setting (Enable)
    0xAF  // Display ON
};

static void oled_set_cursor(struct i2c_client *client, int page, int col) {
    unsigned char cmds[] = {
        0x00,           // Command Stream
        0xB0 + page,    // Set Page Start Address
        0x00 + (col & 0x0F),      // Set Lower Column Start Address
        0x10 + ((col >> 4) & 0x0F)// Set Higher Column Start Address
    };
    i2c_master_send(client, cmds, sizeof(cmds));
}

static void oled_print_big_char(struct i2c_client *client, int page, int col, int char_idx, int invert) {
    int i;
    unsigned char top_buf[9];
    unsigned char bot_buf[9];
    unsigned char d; 
    
    top_buf[0] = 0x40;
    bot_buf[0] = 0x40;
    
    for(i=0; i<8; i++) {
        // 상단 데이터
        d = big_font[char_idx][i];
        if(invert) d = ~d; // 비트 반전 (0->1, 1->0)
        top_buf[i+1] = d;

        // 하단 데이터
        d = big_font[char_idx][i+8];
        if(invert) d = ~d; // 비트 반전
        bot_buf[i+1] = d;
    }
    
    // 윗부분 그리기
    oled_set_cursor(client, page, col);
    i2c_master_send(client, top_buf, 9);
    
    // 아랫부분 그리기
    oled_set_cursor(client, page + 1, col);
    i2c_master_send(client, bot_buf, 9);
}
static void oled_display_time(struct i2c_client *client, int focus_field) {
    struct timespec64 ts;
    struct tm tm_val;
    char time_str[20];
    int i, char_code, invert;
    
    // 1. 시간 가져오기
    ktime_get_real_ts64(&ts);
    time64_to_tm(ts.tv_sec, 0, &tm_val);
    
    // 2. 포맷팅: "YY-MM-DDHH:MM:SS" (인덱스 0~15)
    snprintf(time_str, sizeof(time_str), "%02d-%02d-%02d%02d:%02d:%02d", 
             (int)(tm_val.tm_year + 1900) % 100,
             (int)(tm_val.tm_mon + 1),
             (int)tm_val.tm_mday,
             (int)tm_val.tm_hour,
             (int)tm_val.tm_min,
             (int)tm_val.tm_sec);
             
    // 3. 날짜 출력 (0번 줄)
    for(i=0; i<8; i++) {
        char c = time_str[i];
        if(c >= '0' && c <= '9') char_code = c - '0';
        else char_code = 11; // '-'
        
        // 반전 로직: 현재 인덱스(i)가 선택된 필드 범위에 속하는지 확인
        invert = 0;
        if (focus_field == 1 && (i==0 || i==1)) invert = 1; // 년
        if (focus_field == 2 && (i==3 || i==4)) invert = 1; // 월
        if (focus_field == 3 && (i==6 || i==7)) invert = 1; // 일

        oled_print_big_char(client, 0, 16 + (i * 12), char_code, invert);
    }

    // 4. 시간 출력 (3번 줄)
    for(i=8; i<16; i++) { 
        char c = time_str[i];
        int x_pos = 8 + ((i-8) * 14); 

        if(c >= '0' && c <= '9') char_code = c - '0';
        else char_code = 10; // ':'
        
        // 반전 로직
        invert = 0;
        if (focus_field == 4 && (i==8 || i==9))   invert = 1; // 시
        if (focus_field == 5 && (i==11 || i==12)) invert = 1; // 분
        if (focus_field == 6 && (i==14 || i==15)) invert = 1; // 초

        oled_print_big_char(client, 3, x_pos, char_code, invert);
    }
}
static void oled_clear(struct i2c_client *client) {
    int page;
    unsigned char buf[129]; // 제어바이트 1개 + 데이터 128개 (한 줄 분량)

    // 버퍼 준비: [0x40, 0x00, 0x00, ... 0x00]
    buf[0] = 0x40; // Data Mode
    memset(buf + 1, 0x00, 128); // 나머지는 모두 0으로 채움

    // 0~7페이지(Page)를 돌면서 한 줄씩 싹 지움
    for(page = 0; page < 8; page++) {
        // 커서를 해당 페이지의 맨 앞으로 이동 (이미 만드신 함수 사용)
        oled_set_cursor(client, page, 0);
        
        // 검은색 데이터(0x00) 128개를 한 번에 전송
        i2c_master_send(client, buf, 129);
    }
}

/* 전역 변수로 관리할 클라이언트 포인터 */
static struct i2c_client *oled_client_device = NULL;

static int oled_write_cmds(struct i2c_client *client, const unsigned char *buf, int len) {
    return i2c_master_send(client, buf, len);
}

static int oled_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    pr_info("%s: Device initialized.\n", DRIVER_NAME);
    
    oled_write_cmds(client, oled_init_cmds, sizeof(oled_init_cmds));
    
    // 2. 화면 깨끗이 지우기 (여기 추가!)
    oled_clear(client);
    
    // 3. 시간 표시
    oled_display_time(client, 6);
    return 0;
}

/* 커널 6.x 이상을 위해 void 반환으로 수정됨 */
static void oled_remove(struct i2c_client *client) {
    unsigned char off_cmd[] = {0x00, 0xAE};
    oled_write_cmds(client, off_cmd, sizeof(off_cmd));
    pr_info("%s: Device removed\n", DRIVER_NAME);
}

static const struct i2c_device_id oled_id[] = {
    { "dol12864", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, oled_id);

static struct i2c_driver oled_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = oled_probe,
    .remove = oled_remove,
    .id_table = oled_id,
};

/* -------------------------------------------------------------------------- */
/* 여기서부터가 변경된 핵심 부분입니다: 모듈 로드 시 장치 강제 등록 로직 */
/* -------------------------------------------------------------------------- */

static int __init oled_driver_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info;
    int ret;

    pr_info("%s: Module loading...\n", DRIVER_NAME);

    // 1. 드라이버 먼저 등록
    ret = i2c_add_driver(&oled_driver);
    if (ret < 0) {
        pr_err("%s: Failed to register driver\n", DRIVER_NAME);
        return ret;
    }

    // 2. I2C 어댑터(컨트롤러) 1번 가져오기
    adapter = i2c_get_adapter(I2C_BUS_NUMBER);
    if (!adapter) {
        pr_err("%s: Failed to get I2C adapter %d\n", DRIVER_NAME, I2C_BUS_NUMBER);
        // 어댑터를 못 찾으면 드라이버 등록도 취소
        i2c_del_driver(&oled_driver);
        return -ENODEV;
    }

    // 3. 장치 정보 생성 (이름은 id_table과 같아야 함)
    memset(&board_info, 0, sizeof(struct i2c_board_info));
    strscpy(board_info.type, "dol12864", I2C_NAME_SIZE);
    board_info.addr = OLED_I2C_ADDR;

    // 4. 새로운 장치(Client) 생성 및 등록
    // 이 함수가 호출되면 자동으로 probe 함수가 실행됩니다.
    oled_client_device = i2c_new_client_device(adapter, &board_info);
    
    // 어댑터 사용이 끝났으므로 참조 카운트 감소 (필수)
    i2c_put_adapter(adapter);

    if (IS_ERR(oled_client_device)) {
        pr_err("%s: Failed to create new I2C client\n", DRIVER_NAME);
        i2c_del_driver(&oled_driver);
        return PTR_ERR(oled_client_device);
    }

    pr_info("%s: I2C Client device created successfully\n", DRIVER_NAME);
    return 0;
}

static void __exit oled_driver_exit(void)
{
    // 1. 강제로 등록했던 장치 제거
    if (oled_client_device) {
        i2c_unregister_device(oled_client_device);
    }

    // 2. 드라이버 제거
    i2c_del_driver(&oled_driver);
    pr_info("%s: Module unloaded\n", DRIVER_NAME);
}

module_init(oled_driver_init);
module_exit(oled_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("User");