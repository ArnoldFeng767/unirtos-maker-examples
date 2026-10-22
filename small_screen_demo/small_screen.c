/*****************************************************************/ /**
 * @file small_screen.c
 * @brief 圆屏多功能小工具 Demo（表盘时间 + 秒表 + 番茄钟 + 木鱼计数，基于 GC9A01 SPI 驱动 + RTC）
 *
 * 界面渲染基于 LVGL v8.3.11（unirtos-lvgl 外部库），不再使用自绘位图字体。
 *
 * 屏幕驱动部分（引脚、初始化命令表、读写时序）与 gc9a01.txt 中已验证
 * 可用的实现保持一致。
 **********************************************************************/
#include "qosa_def.h"
#include "qosa_sys.h"
#include "qosa_log.h"
#include "qosa_spi.h"
#include "qosa_gpio.h"
#include "qosa_iic.h"
#include "qosa_pinctrl.h"
#include "qosa_dev_eigen.h"
#include "qosa_rtc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QOS_LOG_TAG   LOG_TAG_DEMO

#include "unirtos_app_init_registry.h"

/* ------------- LCD 接线配置：与 gc9a01.txt 保持一致 ------------- */
#define UNIR_SPI_PORT           (1)
#define UNIR_SPI_CS_PIN         (64)
#define UNIR_SPI_MOSI_PIN       (63)
#define UNIR_SPI_MISO_PIN       (62)
#define UNIR_SPI_CLK_PIN        (49)

#define UNIR_SPI_RST_PIN        (22)
#define UNIR_SPI_DC_PIN         (23)
#define UNIR_SPI_BL_PIN         (19)

#define UNIR_SPI_CS_FUNC         1
#define UNIR_SPI_MOSI_FUNC       1
#define UNIR_SPI_MISO_FUNC       1
#define UNIR_SPI_CLK_FUNC        1

#define UNIR_SPI_RST_FUNC        0
#define UNIR_SPI_DC_FUNC         0
#define UNIR_SPI_BL_FUNC         0

#define LCD_WIDTH               240
#define LCD_HEIGHT              240
#define LCD_ROTATION             0x00

/* RGB565 颜色：纯 C 绘制阶段（unir_spilcd_clear 清屏）仅用到黑色，其余颜色均改用
   LVGL 的 UI_LV_WHITE()/UI_LV_GRAY()/UI_LV_YELLOW() 宏（见后文 LVGL 显示端部分） */
#define COLOR_BLACK              0x0000

/* ------------- 触摸(CST816T)接线与寄存器配置：与 CST816T.txt 保持一致 ------------- */
#define TP_IIC_PORT       0
#define TP_REST_PIN      66
#define TP_REST_FUNC      0
#define TP_INT_PIN       67
#define TP_INT_FUNC       0
#define TP_IIC_SCL       57
#define TP_IIC_SDA       58
#define TP_IIC_SCL_FUNC   2
#define TP_IIC_SDA_FUNC   2

#define TP_REG_CHIP_ID        0xA7
#define TP_REG_VERSION        0xA9
#define TP_CHIP_ID_CST816T    0xB5
#define TP_MACHINE_ADDRESS    0x15

#define TP_REG_GESTURE        0x01
#define TP_GESTURE_NONE       0x00
#define TP_GESTURE_UP         0x01
#define TP_GESTURE_DOWN       0x02
#define TP_GESTURE_LEFT       0x03
#define TP_GESTURE_RIGHT      0x04
#define TP_GESTURE_CLICK      0x05
#define TP_GESTURE_DCLICK     0x0B
#define TP_GESTURE_LONGPRESS  0x0C

#define TP_REG_XPOS_H         0x03
#define TP_REG_XPOS_L         0x04
#define TP_REG_YPOS_H         0x05
#define TP_REG_YPOS_L         0x06

#define TP_REG_IRQ_CTL        0xFA
#define TP_IRQ_MODE_MIXED     0x71
#define TP_IRQ_MODE_GESTURE   0x11
#define TP_REG_NOR_SCAN_PER   0xEE
#define TP_REG_IRQ_PULSE_W    0xED
#define TP_REG_DIS_SLEEP      0xFE

/* 手势识别成功后的冷却时间，单位 ms。CST816T 在一次滑动过程中可能对同一个手势连续触发多次
   INT 下降沿中断（每次扫描周期都会重新上报一次相同的手势编码），导致一次左右滑动被当成两次
   处理。成功处理一次手势后在这段时间内忽略后续所有新手势，跟下面的 TP_IRQ_MODE_GESTURE
   一起从硬件+软件两个层面解决重复触发问题。 */
#define TP_GESTURE_COOLDOWN_MS 400

/* 触摸芯片长时间(数分钟级)无任何触摸操作后可能自行进入低功耗/休眠状态，导致之前只在
   unir_tp_init() 里下发一次的 "禁止休眠"(DIS_SLEEP) 寄存器失效，表现为番茄钟倒计时跑完的
   DONE 页面完全不响应触摸(下滑/双击都没反应)，因为芯片在倒计时期间一直没人碰屏幕。
   解决办法：在轮询函数里每隔 TP_KEEPALIVE_MS 重新下发一次该寄存器，持续保持芯片不休眠。 */
#define TP_KEEPALIVE_MS 2000

/* ---------------------------------------------------------------------------
 * GC9A01 初始化命令表（与 gc9a01.txt 完全一致，已验证可点亮本项目使用的屏幕）
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t        cmd;
    uint8_t        len;
    const uint8_t *param;
    uint16_t       delay_ms;
} unir_lcd_cmd_t;

#define UNIR_LCD_CMD(cmd)             { (uint8_t)(cmd), 0, QOSA_NULL, 0 }
#define UNIR_LCD_CMD_DATA(cmd, ...)                                        \
    {                                                                      \
        (uint8_t)(cmd), (uint8_t)sizeof((const uint8_t[]){__VA_ARGS__}),   \
            (const uint8_t[]){__VA_ARGS__}, 0                              \
    }
#define UNIR_LCD_CMD_D(cmd, delay_ms)                                      \
    { (uint8_t)(cmd), 0, QOSA_NULL, (uint16_t)(delay_ms) }
#define UNIR_LCD_CMD_DATA_D(cmd, delay_ms, ...)                            \
    {                                                                      \
        (uint8_t)(cmd), (uint8_t)sizeof((const uint8_t[]){__VA_ARGS__}),   \
            (const uint8_t[]){__VA_ARGS__}, (uint16_t)(delay_ms)           \
    }

static const unir_lcd_cmd_t gc9a01_init_cmds[] =
{
    UNIR_LCD_CMD_DATA(0x36, LCD_ROTATION),
    UNIR_LCD_CMD(0xEF),
    UNIR_LCD_CMD_DATA(0xEB, 0x14),
    UNIR_LCD_CMD(0xFE),
    UNIR_LCD_CMD(0xEF),
    UNIR_LCD_CMD_DATA(0xEB, 0x14),
    UNIR_LCD_CMD_DATA(0x84, 0x40),
    UNIR_LCD_CMD_DATA(0x85, 0xFF),
    UNIR_LCD_CMD_DATA(0x86, 0xFF),
    UNIR_LCD_CMD_DATA(0x87, 0xFF),
    UNIR_LCD_CMD_DATA(0x88, 0x0A),
    UNIR_LCD_CMD_DATA(0x89, 0x21),
    UNIR_LCD_CMD_DATA(0x8A, 0x00),
    UNIR_LCD_CMD_DATA(0x8B, 0x80),
    UNIR_LCD_CMD_DATA(0x8C, 0x01),
    UNIR_LCD_CMD_DATA(0x8D, 0x01),
    UNIR_LCD_CMD_DATA(0x8E, 0xFF),
    UNIR_LCD_CMD_DATA(0x8F, 0xFF),
    UNIR_LCD_CMD_DATA(0xB6, 0x00, 0x20),
    UNIR_LCD_CMD_DATA(0x3A, 0x05),
    UNIR_LCD_CMD_DATA(0x90, 0x08, 0x08, 0x08, 0x08),
    UNIR_LCD_CMD_DATA(0xBD, 0x06),
    UNIR_LCD_CMD_DATA(0xBC, 0x00),
    UNIR_LCD_CMD_DATA(0xFF, 0x60, 0x01, 0x04),
    UNIR_LCD_CMD_DATA(0xC3, 0x13),
    UNIR_LCD_CMD_DATA(0xC4, 0x13),
    UNIR_LCD_CMD_DATA(0xC9, 0x22),
    UNIR_LCD_CMD_DATA(0xBE, 0x11),
    UNIR_LCD_CMD_DATA(0xE1, 0x10, 0x0E),
    UNIR_LCD_CMD_DATA(0xDF, 0x21, 0x0C, 0x02),
    UNIR_LCD_CMD_DATA(0xF0, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A),
    UNIR_LCD_CMD_DATA(0xF1, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F),
    UNIR_LCD_CMD_DATA(0xF2, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A),
    UNIR_LCD_CMD_DATA(0xF3, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F),
    UNIR_LCD_CMD_DATA(0xED, 0x1B, 0x0B),
    UNIR_LCD_CMD_DATA(0xAE, 0x77),
    UNIR_LCD_CMD_DATA(0xCD, 0x63),
    UNIR_LCD_CMD_DATA(0x70, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03),
    UNIR_LCD_CMD_DATA(0xE8, 0x34),
    UNIR_LCD_CMD_DATA(0x62, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70),
    UNIR_LCD_CMD_DATA(0x63, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70),
    UNIR_LCD_CMD_DATA(0x64, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07),
    UNIR_LCD_CMD_DATA(0x66, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00),
    UNIR_LCD_CMD_DATA(0x67, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98),
    UNIR_LCD_CMD_DATA(0x74, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00),
    UNIR_LCD_CMD_DATA(0x98, 0x3E, 0x07),
    UNIR_LCD_CMD(0x35),
    UNIR_LCD_CMD(0x21),
    UNIR_LCD_CMD_D(0x11, 80),
};

static qosa_gpio_num_e g_lcd_dc_gpio  = QOSA_GPIO_MAX;
static qosa_gpio_num_e g_lcd_rst_gpio = QOSA_GPIO_MAX;
static qosa_gpio_num_e g_lcd_bl_gpio  = QOSA_GPIO_MAX;

/* 写一条 MIPI DCS 命令：DC=0 先发 cmd，再切 DC=1 发参数（无参数则跳过） */
static void unir_spilcd_write(uint8_t cmd, const uint8_t *data, int len)
{
    qosa_gpio_set_level(g_lcd_dc_gpio, 0);
    qosa_spi_write(UNIR_SPI_PORT, &cmd, 1);

    if (data != QOSA_NULL && len > 0)
    {
        qosa_gpio_set_level(g_lcd_dc_gpio, 1);
        qosa_spi_write(UNIR_SPI_PORT, (void *)data, len);
    }
}

/* 设置绘图窗口：0x2A 列地址、0x2B 行地址、0x2C 进入显存写入 */
static void unir_spilcd_setwindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    uint8_t col_param[4] = {0x00, (uint8_t)Xstart, 0x00, (uint8_t)Xend};
    uint8_t row_param[4] = {0x00, (uint8_t)Ystart, 0x00, (uint8_t)Yend};

    unir_spilcd_write(0x2A, col_param, 4);
    unir_spilcd_write(0x2B, row_param, 4);
    unir_spilcd_write(0x2C, QOSA_NULL, 0);
}

/* 在指定窗口内显示图像数据（RGB565，大端字节序） */
static int unir_spilcd_display(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, unsigned char *Image, int len)
{
    unir_spilcd_setwindows(Xstart, Ystart, Xend, Yend);

    qosa_gpio_set_level(g_lcd_dc_gpio, 1);
    qosa_spi_write(UNIR_SPI_PORT, (void *)Image, len);

    return 0;
}

static void unir_spilcd_display_on(void)
{
    unir_spilcd_write(0x29, QOSA_NULL, 0);
    qosa_gpio_set_level(g_lcd_bl_gpio, 1);
}

/* 用纯色填充一个矩形区域（分块发送，避免一次性分配大内存） */
static void unir_spilcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint8_t chunk[512];
    int total = (int)w * (int)h * 2;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    for (unsigned int i = 0; i < sizeof(chunk); i += 2)
    {
        chunk[i]     = hi;
        chunk[i + 1] = lo;
    }

    unir_spilcd_setwindows(x, y, x + w - 1, y + h - 1);
    qosa_gpio_set_level(g_lcd_dc_gpio, 1);

    for (int sent = 0; sent < total; sent += (int)sizeof(chunk))
    {
        int len = (int)sizeof(chunk);
        if (sent + len > total)
        {
            len = total - sent;
        }

        qosa_spi_write(UNIR_SPI_PORT, chunk, len);
    }
}

static void unir_spilcd_clear(void)
{
    unir_spilcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);
}

static void unir_spilcd_send_cmd_list(const unir_lcd_cmd_t *list, unsigned int count)
{
    for (unsigned int i = 0; i < count; i++)
    {
        unir_spilcd_write(list[i].cmd, list[i].param, list[i].len);

        if (list[i].delay_ms > 0)
        {
            qosa_task_sleep_ms(list[i].delay_ms);
        }
    }
}

static void unir_spilcd_init(void)
{
    qosa_pin_cfg_t pin_cfg = {0};
    qosa_spi_nss_mode_e nss_mode = QOSA_SPI_NSS_MASTER_HARDWARE;

    qosa_pin_set_func(UNIR_SPI_MOSI_PIN, UNIR_SPI_MOSI_FUNC);
    qosa_pin_set_func(UNIR_SPI_MISO_PIN, UNIR_SPI_MISO_FUNC);
    qosa_pin_set_func(UNIR_SPI_CLK_PIN, UNIR_SPI_CLK_FUNC);
    qosa_pin_set_func(UNIR_SPI_CS_PIN, UNIR_SPI_CS_FUNC);

    if (qosa_spi_init(UNIR_SPI_PORT, QOSA_SPI_TRANSMIT_POLLING, QOSA_SPI_CLK_13MHZ) != QOSA_SPI_SUCCESS)
    {
        return;
    }

    qosa_spi_ioctl(UNIR_SPI_PORT, QOSA_SPI_IOCTL_SET_NSS_MODE, &nss_mode);

    qosa_get_pin_default_cfg(UNIR_SPI_DC_PIN, &pin_cfg);
    g_lcd_dc_gpio = pin_cfg.gpio_num;
    qosa_pin_set_func(pin_cfg.pin_num, UNIR_SPI_DC_FUNC);
    qosa_gpio_init(pin_cfg.gpio_num, QOSA_GPIO_DIRECTION_OUTPUT, QOSA_GPIO_PULL_NONE, QOSA_GPIO_LEVEL_LOW);

    qosa_get_pin_default_cfg(UNIR_SPI_RST_PIN, &pin_cfg);
    g_lcd_rst_gpio = pin_cfg.gpio_num;
    qosa_pin_set_func(pin_cfg.pin_num, UNIR_SPI_RST_FUNC);
    qosa_gpio_init(pin_cfg.gpio_num, QOSA_GPIO_DIRECTION_OUTPUT, QOSA_GPIO_PULL_NONE, QOSA_GPIO_LEVEL_LOW);

    qosa_get_pin_default_cfg(UNIR_SPI_BL_PIN, &pin_cfg);
    g_lcd_bl_gpio = pin_cfg.gpio_num;
    qosa_pin_set_func(pin_cfg.pin_num, UNIR_SPI_BL_FUNC);
    qosa_gpio_init(pin_cfg.gpio_num, QOSA_GPIO_DIRECTION_OUTPUT, QOSA_GPIO_PULL_NONE, QOSA_GPIO_LEVEL_HIGH);

    qosa_gpio_set_level(g_lcd_rst_gpio, 1);
    qosa_task_sleep_ms(100);
    qosa_gpio_set_level(g_lcd_rst_gpio, 0);
    qosa_task_sleep_ms(100);
    qosa_gpio_set_level(g_lcd_rst_gpio, 1);
    qosa_task_sleep_ms(100);

    unir_spilcd_send_cmd_list(gc9a01_init_cmds, (unsigned int)(sizeof(gc9a01_init_cmds) / sizeof(gc9a01_init_cmds[0])));

    unir_spilcd_clear();
    unir_spilcd_display_on();
}

/* ---------------------------------------------------------------------------
 * CST816T 触摸驱动（与 CST816T.txt 中已验证的实现保持一致）
 *
 * 中断回调只置一个标志位，真正的 I2C 读操作放到任务循环里做（I2C 读写是阻塞
 * 调用，不适合在中断上下文执行），这样既安全又能复用同一个任务轮询触摸和刷新时钟。
 * ------------------------------------------------------------------------- */
static qosa_gpio_num_e     g_tp_rst_gpio  = QOSA_GPIO_MAX;
static qosa_gpio_num_e     g_tp_int_gpio  = QOSA_GPIO_MAX;
static volatile qosa_bool_t g_tp_irq_flag = QOSA_FALSE;
static uint32_t             g_tp_cooldown_ms = 0; /* 还剩多少 ms 处于手势冷却期，期间忽略新手势 */
static uint32_t             g_tp_keepalive_ms = 0; /* 距离上次重新下发 DIS_SLEEP 寄存器过了多少 ms */
static uint16_t             g_tp_touch_x  = 0xFFFF; /* 最近一次 CLICK/DCLICK 的触摸坐标，0xFFFF 表示无效 */
static uint16_t             g_tp_touch_y  = 0xFFFF;

static qosa_i2c_error_e unir_tp_read_reg(qosa_uint8_t reg, qosa_uint8_t *val, qosa_uint8_t len)
{
    return qosa_i2c_read(TP_IIC_PORT, TP_MACHINE_ADDRESS, reg, val, len);
}

static qosa_i2c_error_e unir_tp_write_reg(qosa_uint8_t reg, qosa_uint8_t val, qosa_uint8_t len)
{
    return qosa_i2c_write(TP_IIC_PORT, TP_MACHINE_ADDRESS, reg, &val, len);
}

/* 读取最近一次触摸点坐标（CST816T 的坐标寄存器只要有手指接触就会更新，和 IRQ_CTL
   的手势/混合模式无关），XposH/YposH 高 4 位之外的 bit7:6 是触点事件标志，坐标本身
   只取低 12 位。用于区分"点击在 RESET 按钮区域内"还是"点击屏幕其他位置"。 */
static qosa_bool_t unir_tp_read_point(uint16_t *x, uint16_t *y)
{
    qosa_uint8_t buf[4] = {0};

    if (unir_tp_read_reg(TP_REG_XPOS_H, buf, 4) != QOSA_I2C_SUCCESS)
    {
        return QOSA_FALSE;
    }

    *x = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
    *y = (uint16_t)(((buf[2] & 0x0F) << 8) | buf[3]);
    return QOSA_TRUE;
}

/* INT 脚下降沿中断回调：仅置标志位，不做任何 I2C/LOG 操作 */
static void unir_tp_int_isr(void *argv)
{
    (void)argv;
    g_tp_irq_flag = QOSA_TRUE;
}

static int unir_tp_init(void)
{
    qosa_pin_cfg_t   pin_cfg = {0};
    qosa_int_cfg_t   int_cfg = {0};
    qosa_uint8_t     id      = 0;

    qosa_pin_set_func(TP_IIC_SCL, TP_IIC_SCL_FUNC);
    qosa_pin_set_func(TP_IIC_SDA, TP_IIC_SDA_FUNC);
    qosa_i2c_init(TP_IIC_PORT, QOSA_IIC_STANDARD_MODE);

    qosa_get_pin_default_cfg(TP_REST_PIN, &pin_cfg);
    g_tp_rst_gpio = pin_cfg.gpio_num;
    qosa_pin_set_func(pin_cfg.pin_num, TP_REST_FUNC);
    qosa_gpio_init(pin_cfg.gpio_num, QOSA_GPIO_DIRECTION_OUTPUT, QOSA_GPIO_PULL_NONE, QOSA_GPIO_LEVEL_LOW);

    qosa_get_pin_default_cfg(TP_INT_PIN, &pin_cfg);
    g_tp_int_gpio = pin_cfg.gpio_num;
    qosa_pin_set_func(pin_cfg.pin_num, TP_INT_FUNC);

    int_cfg.gpio_num      = g_tp_int_gpio;
    int_cfg.gpio_pull     = QOSA_GPIO_PULL_UP;
    int_cfg.gpio_debounce = QOSA_GPIO_DEBOUNCE_DIS;
    int_cfg.interrupt_cb  = unir_tp_int_isr;
    int_cfg.user_ctx      = QOSA_NULL;
    int_cfg.options       = 0;
    qosa_interrupt_register(&int_cfg);
    qosa_interrupt_enable(g_tp_int_gpio, QOSA_GPIO_TRIGGER_FALLING_EDGE);

    qosa_gpio_set_level(g_tp_rst_gpio, QOSA_GPIO_LEVEL_HIGH);
    qosa_task_sleep_ms(100);
    qosa_gpio_set_level(g_tp_rst_gpio, QOSA_GPIO_LEVEL_LOW);
    qosa_task_sleep_ms(100);
    qosa_gpio_set_level(g_tp_rst_gpio, QOSA_GPIO_LEVEL_HIGH);
    qosa_task_sleep_ms(100);

    if (unir_tp_read_reg(TP_REG_CHIP_ID, &id, 1) != QOSA_I2C_SUCCESS)
    {
        QLOGE("touch chip id read failed");
        return -1;
    }
    if (id != TP_CHIP_ID_CST816T)
    {
        QLOGE("unexpected touch chip id 0x%02x", id);
    }

    /* 纯手势模式：只在识别到有效手势(包括上下左右滑动/单击/长按)时才辰下降沿中断，
       不像混合模式那样还会因为触摸检测(EN_TOUCH)在滑动过程中每次扫描都触发中断，
       这是之前一次滑动容易被当成两次处理的根本原因之一。坐标寄存器(0x03~0x06)由芯片
       独立更新，不受 IRQ_CTL 模式影响，CLICK/DCLICK 发生时仍可读到触点坐标用于按钮命中判断。 */
    (void)unir_tp_write_reg(TP_REG_IRQ_CTL, TP_IRQ_MODE_GESTURE, 1);
    (void)unir_tp_write_reg(TP_REG_NOR_SCAN_PER, 0x01, 1);
    (void)unir_tp_write_reg(TP_REG_IRQ_PULSE_W, 0x1C, 1);
    (void)unir_tp_write_reg(TP_REG_DIS_SLEEP, 0x01, 1);

    return 0;
}

/* ---------------------------------------------------------------------------
 * LVGL 显示端口：改用 LVGL v8.3.11 (unirtos-lvgl 外部库) 绘制界面，不再使用
 * 自绘位图字体。GC9A01 SPI 驱动 / CST816T 触摸驱动保持不变，只是把原来
 * "手势 -> 直接操作显存" 的处理方式改成 "手势 -> 操作 LVGL 控件"，渲染统一
 * 交给 LVGL 的 flush 回调完成。
 *
 * 字节序：lv_conf.h 里 LV_COLOR_16_SWAP=0，LVGL 内部像素缓冲区是 CPU 原生字节序
 * (Cortex-M 为小端)，而屏幕 SPI 协议要求 RGB565 按大端字节序写入(与原来自绘字体
 * 时完全一样的要求)，所以 flush 回调里仍然要对每个像素做一次高低字节交换。
 *
 * 内存：lv_conf.h 里 LV_MEM_SIZE 只有 48KB，为避免长时间运行产生内存碎片，
 * 本文件所有页面的控件只在启动时创建一次(unir_ui_build)，后续切换页面只是
 * 显示/隐藏(LV_OBJ_FLAG_HIDDEN)和更新文本/样式，不会动态创建/销毁控件。
 * ------------------------------------------------------------------------- */
#include "lvgl.h"

/* 局部绘图缓冲区：240(宽) x 40(行)，单缓冲区(第二个参数传 NULL)。如果后续发现
   SRAM 紧张导致启动失败，可以调小 LVGL_DRAW_BUF_LINES(比如改成 20)，代价是
   单次 flush 覆盖的区域变小、刷新次数变多。 */
#define LVGL_DRAW_BUF_LINES 40
static lv_color_t         s_lvgl_draw_buf[LCD_WIDTH * LVGL_DRAW_BUF_LINES];
static lv_disp_draw_buf_t s_lvgl_disp_buf;
static lv_disp_drv_t      s_lvgl_disp_drv;

/* LVGL -> 屏幕：把 color_p 指向的像素块交换字节序后直接复用现有的 unir_spilcd_display() */
static void unir_lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    int32_t  w     = area->x2 - area->x1 + 1;
    int32_t  h     = area->y2 - area->y1 + 1;
    int32_t  count = w * h;
    uint8_t *bytes = (uint8_t *)color_p;

    for (int32_t i = 0; i < count; i++)
    {
        uint8_t tmp      = bytes[i * 2];
        bytes[i * 2]     = bytes[i * 2 + 1];
        bytes[i * 2 + 1] = tmp;
    }

    unir_spilcd_display((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)area->x2, (uint16_t)area->y2,
                         bytes, (int)(count * 2));

    lv_disp_flush_ready(disp_drv);
}

static void unir_lvgl_disp_init(void)
{
    lv_disp_draw_buf_init(&s_lvgl_disp_buf, s_lvgl_draw_buf, QOSA_NULL, LCD_WIDTH * LVGL_DRAW_BUF_LINES);

    lv_disp_drv_init(&s_lvgl_disp_drv);
    s_lvgl_disp_drv.hor_res  = LCD_WIDTH;
    s_lvgl_disp_drv.ver_res  = LCD_HEIGHT;
    s_lvgl_disp_drv.flush_cb = unir_lvgl_flush_cb;
    s_lvgl_disp_drv.draw_buf = &s_lvgl_disp_buf;
    lv_disp_drv_register(&s_lvgl_disp_drv);
}

/* 常用颜色（函数形式，只能在函数体内调用，不能用于静态初始化） */
#define UI_LV_WHITE()   lv_color_white()
#define UI_LV_BLACK()   lv_color_black()
#define UI_LV_GRAY()    lv_color_hex(0xA0A0A0)
#define UI_LV_YELLOW()  lv_color_hex(0xFFE000) /* 木鱼计数点击高亮动效 / 清零确认提示用 */

/* 构造一个铺满整个圆屏(240x240)的容器，黑底、无边框圆角、不可滚动，用作每个页面的根节点 */
static void unir_ui_page_container_init(lv_obj_t *obj)
{
    lv_obj_set_size(obj, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, UI_LV_BLACK(), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_radius(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

/* 构造一个满宽(240px)、水平居中对齐、固定 y 坐标的文本标签，复用原来手绘字体版本
   里已经在真实屏幕上调好的坐标，减少重新调试的工作量 */
static lv_obj_t *unir_ui_label_create(lv_obj_t *parent, int y, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_width(label, LCD_WIDTH);
    lv_obj_set_pos(label, 0, y);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label, color, 0);
    if (font != QOSA_NULL)
    {
        lv_obj_set_style_text_font(label, font, 0);
    }

    return label;
}

static const char *const WEEKDAY_NAMES[7] =
{
    "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"
};

/* RTC 返回的是 UTC+时区偏移，但模组默认时区是 0（即纯 UTC），不会自动按北京时间(UTC+8)
   换算，所以必须显式设置一次时区，否则时间会比实际晚 8 小时（例如实际 16:00 显示成 08:00）。
   qosa_rtc_set_timezone 入参单位是 15 分钟（与 3GPP NITZ 时区格式一致），UTC+8 = 8*4 = 32。 */
#define RTC_TIMEZONE_QUARTER_HOURS   32

/* 开机时 RTC 还没来得及从基站/网络同步到真实时间（需要等模组注册网络，有一定延迟），
   在这之前 qosa_rtc_get_localtime 会返回 1970-01-01 近似值，换算成北京时间就是把字屏闪一下显示
   成错误的 "08:00"。用这个阈值判断 RTC 是否已经同步到合理的日期(暂定 2024 年以后)，
   未同步完成前屏幕显示 "SYNCING" 占位，一旦同步完成立即整屏切换成真实时间，
   避免用户看到中间错误的时间摆计。 */
#define RTC_VALID_YEAR_MIN   (2024 - 1900)

/* 触摸轮询任务循环节奏：每 50ms 轮询一次触摸，同时驱动 LVGL 的 lv_tick_inc()/lv_timer_handler()。
   提前到文件靠前面定义是因为 unir_tp_poll() 和各页面的 lv_timer 回调都需要引用这个常量。 */
#define UI_POLL_INTERVAL_MS    50

/* 布局参数：复用手绘字体版本里已经在真实屏幕上调好的坐标，减少重新调试的工作量。
   大号数字统一改用内置的 28px 字体(lv_font_montserrat_28)直接渲染，不再使用
   transform_zoom 缩放 16px 字体——zoom 对 label 这类非图片控件是靠临时申请一块
   离屏缓冲区、按“图片”二次绘制实现的，在 48KB 堆池下几乎总是分配失败，导致放大的
   数字整体不渲染(背景色直接透出来)，和实测现象完全吻合。 */
#define TIME_Y      95  /* 比之前更靠近屏幕垂直中心(120)，配合 40px 字体(行高 44)居中显示 */
#define DATE_Y     155  /* 20px 字体(行高 22)，紧接在时间下方 */
#define WEEK_Y     185  /* 跟着日期变高同步下移，保持与日期间距 */

/* 表盘外圈装饰圆环：仿智能手表表盘风格（参考 a.png），纯装饰、不可交互、不显示任何数值，
   只用两段不同颜色的圆弧勾勒出屏幕边缘，让表盘看起来更有层次感 */
#define CLOCK_RING_SIZE          232                                  /* 圆环直径，留 4px 边距给 240x240 圆屏 */
#define CLOCK_RING_X             ((LCD_WIDTH  - CLOCK_RING_SIZE) / 2)
#define CLOCK_RING_Y             ((LCD_HEIGHT - CLOCK_RING_SIZE) / 2)
#define CLOCK_RING_WIDTH           8                                  /* 圆弧线宽 */

/* ---------------------------------------------------------------------------
 * 页面导航（表盘 / 功能选择页 / 秒表运行页 / 番茄钟运行页 / 木鱼计数运行页 / 计算器运行页）
 *
 * 交互方式：
 *   表盘页      上滑(UP)                  -> 进入功能选择页，同时显示功能图标
 *   功能选择页  上滑(UP)/下滑(DOWN)        -> 上下选择功能：选中项放大居中显示，相邻项缩小
 *                                            显示在上/下方；已在最后一项时上滑无效，已在
 *                                            第一项时下滑则返回表盘页
 *              点击(CLICK/DCLICK)         -> 进入当前选中项对应的独立运行页（秒表/番茄钟/木鱼/计算器）
 *   秒表运行页  点击(CLICK/DCLICK)         -> 开始/停止计时（支持快速连续两次点击）
 *              长按(LONGPRESS)            -> 触发画面上的 RESET 按钮，计时归零
 *              下滑(DOWN)                 -> 返回功能选择页
 *   木鱼计数页  点击(CLICK/DCLICK)         -> 计数+1，放大+高亮动效反馈；清零确认状态下
 *                                            则表示确认清零
 *              长按(LONGPRESS)            -> 进入清零确认状态（需再点一次才真正清零）
 *              下滑(DOWN)                 -> 清零确认状态下表示取消确认；否则返回功能选择页
 *   计算器运行页 点击(CLICK/DCLICK)        -> 按坐标命中 4x4 键盘按钮(7 8 9 ÷ / 4 5 6 × /
 *                                            1 2 3 − / C 0 = +)，支持连续四则运算
 *              下滑(DOWN)                 -> 返回功能选择页（计算器状态保留，再次进入时继续）
 * 功能选择页各功能的圆形图标按选中状态垂直排列(选中项放大居中，相邻项缩小显示在上下方)，
 * 名称固定显示在图标右侧，后续要新增功能时只需要在 s_features[] 里追加一项，布局会自动按
 * 行偏移处理，一般不需要再调整间距。
 * ------------------------------------------------------------------------- */
typedef enum
{
    UI_FEATURE_PLACEHOLDER = 0, /* 占位页：显示 "NOT READY"，具体功能留待后续实现 */
    UI_FEATURE_STOPWATCH,       /* 秒表：轮播卡片只做介绍，点击后进入独立的运行页 */
    UI_FEATURE_TOMATO,          /* 番茄钟：同样轮播卡片只做介绍，点击后进入独立的倒计时运行页 */
    UI_FEATURE_FISH,            /* 木鱼计数器：同样轮播卡片只做介绍，点击后进入独立的计数运行页 */
    UI_FEATURE_CALC,            /* 计算器：同样轮播卡片只做介绍，点击后进入独立的四则运算运行页 */
} ui_feature_kind_e;

typedef struct
{
    const char           *line1;      /* 功能名称第一行 */
    const char           *line2;      /* 功能名称第二行，单行标题填 QOSA_NULL */
    ui_feature_kind_e     kind;       /* 决定点击后进入哪个独立运行页（见 unir_ui_handle_gesture） */
    const lv_img_dsc_t   *icon;       /* 顶部圆形徽标里居中显示的图标图片，见 feature_icons.c */
    uint32_t              accent_hex; /* 该功能的主题色(0xRRGGBB)，使用时需要 lv_color_hex() 转换(不能用于静态初始化) */
} ui_feature_t;

/* 四个图标分别由 tools/gen_feature_icons.py 从 image/a.png(木鱼)、image/d.png(番茄钟)、
   image/e.png(秒表)、image/f.png(计算器) 转换生成，见 feature_icons.c */
extern const lv_img_dsc_t img_icon_fish;
extern const lv_img_dsc_t img_icon_timer;
extern const lv_img_dsc_t img_icon_stopwatch;
extern const lv_img_dsc_t img_icon_calc;

static const ui_feature_t s_features[] =
{
    { "FISH",      "COUNTER", UI_FEATURE_FISH,      &img_icon_fish,      0xFFD166 }, /* 木鱼计数器：金色 + 木鱼图标 */
    { "TIMER",     QOSA_NULL, UI_FEATURE_TOMATO,    &img_icon_timer,     0xFF5A3C }, /* 番茄钟：橙红色 + 番茄钟图标，和表盘页下半环颜色呼应 */
    { "STOPWATCH", QOSA_NULL, UI_FEATURE_STOPWATCH, &img_icon_stopwatch, 0x27C4A3 }, /* 秒表：青绿色 + 秒表图标，和表盘页上半环颜色呼应 */
    { "CALC",      QOSA_NULL, UI_FEATURE_CALC,      &img_icon_calc,      0x4A90D9 }, /* 计算器：蓝色 + 计算器图标 */
};

/* 番茄钟/计算器图标圆形徽标的背景填充色：都采样自各自 image/d.png、image/f.png 四周的浅色底色，
   让圆形裁剪边缘处的抗锯齿过渡色与图标本身的背景融为一体，避免与页面黑底产生生硬的深色描边 */
#define FEAT_TOMATO_BADGE_BG_HEX 0xFFF8EB
#define FEAT_CALC_BADGE_BG_HEX   0xFFFFFF
#define UI_FEATURE_COUNT ((int)(sizeof(s_features) / sizeof(s_features[0])))
typedef enum
{
    UI_PAGE_CLOCK = 0,     /* 表盘：时间+日期+星期 */
    UI_PAGE_FEATURE,       /* 功能轮播页：一次只显示一个功能的卡片内容 */
    UI_PAGE_STOPWATCH_RUN, /* 秒表运行页：点击秒表卡片后进入，居中显示计时 */
    UI_PAGE_TOMATO_RUN,    /* 番茄钟运行页：点击 TIMER 卡片后进入，居中显示倒计时 */
    UI_PAGE_FISH_RUN,      /* 木鱼计数运行页：点击 FISH 卡片后进入，居中显示计数 */
    UI_PAGE_CALC_RUN,      /* 计算器运行页：点击 CALC 卡片后进入，显示数字输入框+4x4 键盘 */
} ui_page_e;

static ui_page_e   g_ui_page       = UI_PAGE_CLOCK;

/* ---------------- 表盘页：容器 + 2 段装饰圆弧 + 4 个标签，lv_timer 每秒刷新一次 ---------------- */
static lv_obj_t    *p_clock;
static lv_obj_t    *arc_clock_top;
static lv_obj_t    *arc_clock_bottom;
static lv_obj_t    *lbl_clock_time;
static lv_obj_t    *lbl_clock_date;
static lv_obj_t    *lbl_clock_week;
static lv_timer_t  *s_clock_timer;

/* lv_timer 回调（周期 1000ms）：读 RTC，未同步完成前显示 "SYNCING" 占位（缩小字号），
   同步完成后恢复大号时间 + 日期 + 星期显示。也会在 unir_ui_show_page(UI_PAGE_CLOCK) 时
   被手动调用一次(timer 参数传 QOSA_NULL)，确保刚进入表盘页就是最新内容，不用等满 1s。 */
static void unir_ui_refresh_clock(lv_timer_t *timer)
{
    qosa_time_t     utc_time = 0;
    qosa_rtc_time_t tm       = {0};
    char            time_str[8];
    char            date_str[8];

    (void)timer;

    if (qosa_rtc_get_localtime(&utc_time) != 0 || qosa_rtc_gmtime_r(&utc_time, &tm) == QOSA_NULL)
    {
        return;
    }

    if (tm.tm_year < RTC_VALID_YEAR_MIN)
    {
        lv_label_set_text(lbl_clock_time, "SYNCING");
        lv_obj_add_flag(lbl_clock_date, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lbl_clock_week, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(lbl_clock_date, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_clock_week, LV_OBJ_FLAG_HIDDEN);

    snprintf(time_str, sizeof(time_str), "%02d:%02d", tm.tm_hour, tm.tm_min);
    snprintf(date_str, sizeof(date_str), "%02d/%02d", tm.tm_mon + 1, tm.tm_mday);

    lv_label_set_text(lbl_clock_time, time_str);
    lv_label_set_text(lbl_clock_date, date_str);

    if (tm.tm_wday >= 0 && tm.tm_wday <= 6)
    {
        lv_label_set_text(lbl_clock_week, WEEKDAY_NAMES[tm.tm_wday]);
    }
}

/* 把一个 lv_arc 配置成纯装饰用的圆弧段：隐藏灰色底环和拖拽圆钮，只保留从 start 到
   end 角度(LVGL 角度制，0°=3点钟方向，顺时针递增)的彩色圆弧，不响应触摸 */
static void unir_ui_style_decor_arc(lv_obj_t *arc, uint16_t start, uint16_t end, lv_color_t color)
{
    lv_obj_remove_style(arc, QOSA_NULL, LV_PART_KNOB | LV_STATE_ANY);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, CLOCK_RING_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
    lv_arc_set_bg_angles(arc, start, end);
    lv_arc_set_angles(arc, start, end);
}

static void unir_ui_build_clock_page(void)
{
    p_clock = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_clock);

    /* 两段装饰圆弧：青绿色覆盖屏幕上半部分，橙红色覆盖下半部分，中间留出左右两道缝隙，
       视觉上呼应智能手表表盘常见的双色进度环设计 */
    arc_clock_top = lv_arc_create(p_clock);
    lv_obj_set_size(arc_clock_top, CLOCK_RING_SIZE, CLOCK_RING_SIZE);
    lv_obj_set_pos(arc_clock_top, CLOCK_RING_X, CLOCK_RING_Y);
    unir_ui_style_decor_arc(arc_clock_top, 200, 340, lv_color_hex(0x27C4A3));

    arc_clock_bottom = lv_arc_create(p_clock);
    lv_obj_set_size(arc_clock_bottom, CLOCK_RING_SIZE, CLOCK_RING_SIZE);
    lv_obj_set_pos(arc_clock_bottom, CLOCK_RING_X, CLOCK_RING_Y);
    unir_ui_style_decor_arc(arc_clock_bottom, 20, 160, lv_color_hex(0xFF5A3C));

    lbl_clock_time = unir_ui_label_create(p_clock, TIME_Y, &lv_font_montserrat_40, UI_LV_WHITE());
    lbl_clock_date = unir_ui_label_create(p_clock, DATE_Y, &lv_font_montserrat_20, UI_LV_GRAY());
    lbl_clock_week = unir_ui_label_create(p_clock, WEEK_Y, QOSA_NULL, UI_LV_GRAY());

    s_clock_timer = lv_timer_create(unir_ui_refresh_clock, 1000, QOSA_NULL);
}

/* ---------------- 秒表运行页：容器 + 计时/状态标签 + RESET 按钮 + 提示，lv_timer 每 50ms 计时一次 ---------------- */
static qosa_bool_t g_sw_running    = QOSA_FALSE;
static uint32_t    g_sw_elapsed_ms = 0;

#define SW_TIME_Y          95  /* 计时数字：放在屏幕正中间（视觉居中，为下方状态/按钮/提示留出空间） */
#define SW_STATUS_Y       140 /* RUNNING/PAUSED 状态文字 */
#define SW_RESET_BTN_W    100
#define SW_RESET_BTN_H     28
#define SW_RESET_BTN_Y    165 /* 画成按钮样式的 RESET（灰底黑字） */
#define SW_RESET_BTN_X    ((LCD_WIDTH - SW_RESET_BTN_W) / 2)
#define SW_HIT_MARGIN      10 /* 按钮命中区域在绘制区域基础上四周各扩大这么多像素，方便点击 */
#define SW_BTN_FLASH_MS   120 /* 点中 RESET 按钮时短暂进入 LVGL PRESSED 态的视觉反馈时长 */

static lv_obj_t    *p_sw;
static lv_obj_t    *lbl_sw_time;
static lv_obj_t    *lbl_sw_status;
static lv_obj_t    *obj_sw_reset_btn;
static lv_obj_t    *lbl_sw_reset_text;
static lv_timer_t  *s_sw_tick_timer;

/* 判断一个触摸坐标是否落在某个控件的命中区域内（比控件实际区域四周各放宽 margin 像素） */
static qosa_bool_t unir_ui_point_in_obj(lv_obj_t *obj, int margin, uint16_t x, uint16_t y)
{
    lv_area_t area;

    lv_obj_get_coords(obj, &area);

    return (qosa_bool_t)((int)x >= area.x1 - margin && (int)x <= area.x2 + margin &&
                          (int)y >= area.y1 - margin && (int)y <= area.y2 + margin);
}

/* 按钮被点中后短暂进入 LVGL PRESSED 态(默认主题会自动变暗)，制造一个轻量点击反馈动效，
   复用 LVGL 自带的状态样式而不用手动改颜色；一次性 lv_timer 到期后清除状态并自我删除 */
static void unir_ui_btn_flash_revert_cb(lv_timer_t *timer)
{
    lv_obj_t *btn = (lv_obj_t *)timer->user_data;

    lv_obj_clear_state(btn, LV_STATE_PRESSED);
    lv_timer_del(timer);
}

static void unir_ui_btn_flash(lv_obj_t *btn)
{
    lv_timer_t *t;

    lv_obj_add_state(btn, LV_STATE_PRESSED);
    t = lv_timer_create(unir_ui_btn_flash_revert_cb, SW_BTN_FLASH_MS, btn);
    lv_timer_set_repeat_count(t, 1);
}

static void unir_ui_sw_update_time_label(void)
{
    uint32_t total_sec   = g_sw_elapsed_ms / 1000;
    uint32_t minutes     = (total_sec / 60) % 100;
    uint32_t seconds     = total_sec % 60;
    uint32_t centisecond = (g_sw_elapsed_ms % 1000) / 10;

    lv_label_set_text_fmt(lbl_sw_time, "%02u:%02u.%02u",
                           (unsigned int)minutes, (unsigned int)seconds, (unsigned int)centisecond);
}

static void unir_ui_sw_tick_cb(lv_timer_t *timer)
{
    (void)timer;
    g_sw_elapsed_ms += UI_POLL_INTERVAL_MS;
    unir_ui_sw_update_time_label();
}

static void unir_ui_sw_set_running(qosa_bool_t running)
{
    g_sw_running = running;
    lv_label_set_text(lbl_sw_status, running ? "RUNNING" : "PAUSED");

    if (running)
    {
        lv_timer_resume(s_sw_tick_timer);
    }
    else
    {
        lv_timer_pause(s_sw_tick_timer);
    }
}

static void unir_ui_sw_reset(void)
{
    g_sw_elapsed_ms = 0;
    unir_ui_sw_update_time_label();
    unir_ui_sw_set_running(QOSA_FALSE);
}

static void unir_ui_build_stopwatch_page(void)
{
    p_sw = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_sw);
    lv_obj_add_flag(p_sw, LV_OBJ_FLAG_HIDDEN);

    lbl_sw_time = unir_ui_label_create(p_sw, SW_TIME_Y, &lv_font_montserrat_28, UI_LV_WHITE());

    lbl_sw_status = unir_ui_label_create(p_sw, SW_STATUS_Y, QOSA_NULL, UI_LV_GRAY());

    obj_sw_reset_btn = lv_btn_create(p_sw);
    lv_obj_set_size(obj_sw_reset_btn, SW_RESET_BTN_W, SW_RESET_BTN_H);
    lv_obj_set_pos(obj_sw_reset_btn, SW_RESET_BTN_X, SW_RESET_BTN_Y);
    lv_obj_clear_flag(obj_sw_reset_btn, LV_OBJ_FLAG_CLICKABLE); /* 不走 LVGL 自带点击事件，命中判断由触摸手势代码手动完成 */
    lv_obj_set_style_bg_color(obj_sw_reset_btn, UI_LV_GRAY(), 0);
    lv_obj_set_style_radius(obj_sw_reset_btn, 6, 0);

    lbl_sw_reset_text = lv_label_create(obj_sw_reset_btn);
    lv_label_set_text(lbl_sw_reset_text, "RESET");
    lv_obj_set_style_text_color(lbl_sw_reset_text, UI_LV_BLACK(), 0);
    lv_obj_center(lbl_sw_reset_text);

    s_sw_tick_timer = lv_timer_create(unir_ui_sw_tick_cb, UI_POLL_INTERVAL_MS, QOSA_NULL);
    lv_timer_pause(s_sw_tick_timer);

    unir_ui_sw_update_time_label();
    lv_label_set_text(lbl_sw_status, "PAUSED");
}

/* ---------------------------------------------------------------------------
 * 番茄钟（TIMER 卡片）：倒计时，单击开始/暂停，长按复位，倒计时跑完后切换到
 * 白底黑字的醒目完成界面。改用 LVGL 控件后额外加了一个 lv_bar 展示剩余时间比例。
 * ------------------------------------------------------------------------- */
#define TOMATO_DEFAULT_MS (25 * 60 * 1000)  /* 默认倒计时时长：25 分钟(经典番茄钟工作法时长) */
#define TOMATO_STEP_MS     (1 * 60 * 1000)  /* 左右滑动调整时长的步进：每次 1 分钟 */
#define TOMATO_MIN_MS      (1 * 60 * 1000)  /* 最短可设置 1 分钟 */
#define TOMATO_MAX_MS     (60 * 60 * 1000)  /* 最长可设置 60 分钟 */

#define TOMATO_TIME_Y      95  /* 倒计时数字：和秒表一样居中显示 */
#define TOMATO_BAR_Y      140
#define TOMATO_BAR_W      180
#define TOMATO_BAR_H       10
#define TOMATO_BAR_X      ((LCD_WIDTH - TOMATO_BAR_W) / 2)
#define TOMATO_STATUS_Y   160

static qosa_bool_t g_tomato_running      = QOSA_FALSE;
static qosa_bool_t g_tomato_done         = QOSA_FALSE; /* 倒计时是否已经跑完，跑完后单击不再重新开始，只能长按复位 */
static uint32_t    g_tomato_duration_ms  = TOMATO_DEFAULT_MS; /* 用户自定义的倒计时总时长，复位时恢复到这个值而不是固定的默认值 */
static uint32_t    g_tomato_remaining_ms = TOMATO_DEFAULT_MS;

static lv_obj_t    *p_tomato;
static lv_obj_t    *lbl_tomato_time;
static lv_obj_t    *bar_tomato;
static lv_obj_t    *lbl_tomato_status;
static lv_timer_t  *s_tomato_tick_timer;

/* 根据当前状态(运行中/暂停/已完成)刷新倒计时数字、进度条、状态文字、提示文案和容器底色。
   跑完后把容器切换成白底黑字、隐藏进度条和状态文字，只显示放大的 "DONE"，和原来手绘版的
   "整屏反色" 效果保持一致。 */
static void unir_ui_tomato_refresh(void)
{
    uint32_t total_sec = g_tomato_remaining_ms / 1000;
    uint32_t minutes   = (total_sec / 60) % 100;
    uint32_t seconds   = total_sec % 60;

    if (g_tomato_done)
    {
        lv_obj_set_style_bg_color(p_tomato, UI_LV_WHITE(), 0);
        lv_obj_set_style_text_color(lbl_tomato_time, UI_LV_BLACK(), 0);
        lv_label_set_text(lbl_tomato_time, "DONE");
        lv_obj_add_flag(bar_tomato, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lbl_tomato_status, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_set_style_bg_color(p_tomato, UI_LV_BLACK(), 0);
    lv_obj_set_style_text_color(lbl_tomato_time, UI_LV_WHITE(), 0);
    lv_label_set_text_fmt(lbl_tomato_time, "%02u:%02u", (unsigned int)minutes, (unsigned int)seconds);
    lv_obj_clear_flag(bar_tomato, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_tomato_status, LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_range(bar_tomato, 0, (int32_t)g_tomato_duration_ms);
    lv_bar_set_value(bar_tomato, (int32_t)g_tomato_remaining_ms, LV_ANIM_OFF);
    lv_label_set_text(lbl_tomato_status, g_tomato_running ? "RUNNING" : "PAUSED");
}

/* lv_timer 回调(周期 1000ms，按秒倒数，精度足够 MM:SS 显示)：减到 0 就停止 + 标记完成 */
static void unir_ui_tomato_tick_cb(lv_timer_t *timer)
{
    (void)timer;

    if (g_tomato_remaining_ms > 1000)
    {
        g_tomato_remaining_ms -= 1000;
        unir_ui_tomato_refresh();
    }
    else
    {
        g_tomato_remaining_ms = 0;
        g_tomato_running      = QOSA_FALSE;
        g_tomato_done         = QOSA_TRUE;
        lv_timer_pause(s_tomato_tick_timer);
        unir_ui_tomato_refresh();
    }
}

static void unir_ui_tomato_set_running(qosa_bool_t running)
{
    g_tomato_running = running;

    if (running)
    {
        lv_timer_resume(s_tomato_tick_timer);
    }
    else
    {
        lv_timer_pause(s_tomato_tick_timer);
    }

    unir_ui_tomato_refresh();
}

static void unir_ui_tomato_reset(void)
{
    g_tomato_running      = QOSA_FALSE;
    g_tomato_done         = QOSA_FALSE;
    g_tomato_remaining_ms = g_tomato_duration_ms;
    lv_timer_pause(s_tomato_tick_timer);
    unir_ui_tomato_refresh();
}

/* 调整自定义时长（仅在未开始/已暂停且未完成时生效，避免运行中被误触改时长）：
   delta_min 为正负分钟数，调整后夹在 [TOMATO_MIN_MS, TOMATO_MAX_MS] 范围内，
   并同步将剩余时间重置为新的总时长 */
static void unir_ui_tomato_adjust(int delta_min)
{
    int32_t new_ms;

    if (g_tomato_running || g_tomato_done)
    {
        return;
    }

    new_ms = (int32_t)g_tomato_duration_ms + delta_min * (int32_t)TOMATO_STEP_MS;
    if (new_ms < (int32_t)TOMATO_MIN_MS)
    {
        new_ms = (int32_t)TOMATO_MIN_MS;
    }
    else if (new_ms > (int32_t)TOMATO_MAX_MS)
    {
        new_ms = (int32_t)TOMATO_MAX_MS;
    }

    g_tomato_duration_ms  = (uint32_t)new_ms;
    g_tomato_remaining_ms = g_tomato_duration_ms;
    unir_ui_tomato_refresh();
}

static void unir_ui_build_tomato_page(void)
{
    p_tomato = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_tomato);
    lv_obj_add_flag(p_tomato, LV_OBJ_FLAG_HIDDEN);

    lbl_tomato_time = unir_ui_label_create(p_tomato, TOMATO_TIME_Y, &lv_font_montserrat_28, UI_LV_WHITE());

    bar_tomato = lv_bar_create(p_tomato);
    lv_obj_set_size(bar_tomato, TOMATO_BAR_W, TOMATO_BAR_H);
    lv_obj_set_pos(bar_tomato, TOMATO_BAR_X, TOMATO_BAR_Y);
    lv_bar_set_range(bar_tomato, 0, (int32_t)g_tomato_duration_ms);
    lv_obj_set_style_bg_color(bar_tomato, UI_LV_GRAY(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_tomato, UI_LV_WHITE(), LV_PART_INDICATOR);

    lbl_tomato_status = unir_ui_label_create(p_tomato, TOMATO_STATUS_Y, QOSA_NULL, UI_LV_GRAY());

    s_tomato_tick_timer = lv_timer_create(unir_ui_tomato_tick_cb, 1000, QOSA_NULL);
    lv_timer_pause(s_tomato_tick_timer);

    unir_ui_tomato_refresh();
}

/* ---------------------------------------------------------------------------
 * 木鱼计数器（FISH 卡片）：点击屏幕任意位置 +1，并做放大+高亮动效反馈（直接通过修改
 * 标签自身的背景色/字色/缩放倍数实现，而不是像手绘版那样手动清屏重画）；长按触发
 * 清零确认，确认前再点一次才会真正清零，下滑在确认状态下表示取消确认。
 * ------------------------------------------------------------------------- */
extern const lv_img_dsc_t img_muyu_bg; /* 木鱼背景图，由 tools/gen_fish_img.py 从 b.png 转换生成，见 fish_img_muyu.c */

#define FISH_COUNT_Y            185  /* 计数数字：放在木鱼图案下方的留白区域，比之前更靠下 */
#define FISH_ANIM_MS            150  /* 高亮状态维持的时长，之后自动恢复正常显示 */
#define FISH_IMG_ZOOM_NORMAL    256  /* lv_img 缩放基准值，256 = 1倍(不缩放)，定义见 LV_IMG_ZOOM_NONE */
#define FISH_IMG_ZOOM_ANIM      288  /* 点击高亮时的放大倍数，288/256 ≈ 1.125倍 */
#define FISH_HINT_Y             198  /* 操作提示：多行合并显示 */
#define FISH_PLUS1_Y              50  /* "+1" 漂浮动画起始 y 坐标，放在屏幕上半部分 */
#define FISH_PLUS1_RISE_PX        40  /* "+1" 漂浮动画总共上移的像素 */
#define FISH_PLUS1_MS            700  /* "+1" 漂浮动画持续时间（上移 + 渐隐同步进行） */

static uint32_t    g_fish_count           = 0;
static qosa_bool_t g_fish_confirm_pending = QOSA_FALSE; /* 长按后是否处于"再点一次才清零"的确认等待状态 */

static lv_obj_t *p_fish;
static lv_obj_t *img_fish;
static lv_obj_t *lbl_fish_count;
static lv_obj_t *lbl_fish_hint;

/* 刷新计数数字：animate=QOSA_TRUE 时放大并用黄底黑字高亮显示（点击反馈瞬间），
   animate=QOSA_FALSE 时按正常大小用透明底白字显示 */
static void unir_ui_fish_refresh_count(qosa_bool_t animate)
{
    lv_label_set_text_fmt(lbl_fish_count, "%u", (unsigned int)g_fish_count);

    if (animate)
    {
        lv_obj_set_style_bg_opa(lbl_fish_count, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(lbl_fish_count, UI_LV_YELLOW(), 0);
        lv_obj_set_style_text_color(lbl_fish_count, UI_LV_BLACK(), 0);
    }
    else
    {
        lv_obj_set_style_bg_opa(lbl_fish_count, LV_OPA_TRANSP, 0);
        lv_obj_set_style_text_color(lbl_fish_count, UI_LV_WHITE(), 0);
    }
}

/* 刷新木鱼图片自身的缩放状态：animate=QOSA_TRUE 时放大，animate=QOSA_FALSE 时恢复原大小 */
static void unir_ui_fish_refresh_img(qosa_bool_t animate)
{
    if (animate)
    {
        lv_img_set_zoom(img_fish, FISH_IMG_ZOOM_ANIM);
    }
    else
    {
        lv_img_set_zoom(img_fish, FISH_IMG_ZOOM_NORMAL);
    }
}

/* 刷新提示文案：正常状态显示"点击+1/长按清零/下滑返回"，
   清零确认状态下改成醒目的"CLEAR COUNT?"确认提示 */
static void unir_ui_fish_refresh_hints(void)
{
    if (g_fish_confirm_pending)
    {
        lv_obj_set_style_text_color(lbl_fish_hint, UI_LV_YELLOW(), 0);
        lv_label_set_text(lbl_fish_hint, "CLEAR COUNT?\nTAP YES\nDOWN CANCEL");
    }
    else
    {
        /* 空闲状态不再显示操作提示文字，仅在清零确认时才显示上面的提示 */
        lv_label_set_text(lbl_fish_hint, "");
    }
}

/* 一次性 lv_timer：点击高亮动效到期后恢复正常显示，并自我删除 */
static void unir_ui_fish_anim_revert_cb(lv_timer_t *timer)
{
    unir_ui_fish_refresh_count(QOSA_FALSE);
    unir_ui_fish_refresh_img(QOSA_FALSE);
    lv_timer_del(timer);
}

/* lv_anim 的 opa 通道需要三参数的 lv_obj_set_style_opa，这里包一层只接受 (obj, value) 的签名 */
static void unir_ui_fish_plus1_set_opa_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

/* 渐隐动画播放结束后删除这个临时的 "+1" 标签 */
static void unir_ui_fish_plus1_ready_cb(lv_anim_t *a)
{
    lv_obj_del((lv_obj_t *)a->var);
}

/* 每次点击时，在屏幕上半部分生成一个临时的 "+1" 标签，向上漂浮的同时渐渐透明，
   动画结束后自动销毁，与下方常驻的计数数字位置相互独立，互不遮挡 */
static void unir_ui_fish_spawn_plus1(void)
{
    lv_obj_t *lbl;
    lv_anim_t a;

    lbl = unir_ui_label_create(p_fish, FISH_PLUS1_Y, &lv_font_montserrat_28, UI_LV_YELLOW());
    lv_label_set_text(lbl, "+1");
    lv_obj_set_style_opa(lbl, LV_OPA_COVER, 0);

    lv_anim_init(&a);
    lv_anim_set_var(&a, lbl);
    lv_anim_set_time(&a, FISH_PLUS1_MS);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&a, FISH_PLUS1_Y, FISH_PLUS1_Y - FISH_PLUS1_RISE_PX);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    lv_anim_init(&a);
    lv_anim_set_var(&a, lbl);
    lv_anim_set_time(&a, FISH_PLUS1_MS);
    lv_anim_set_exec_cb(&a, unir_ui_fish_plus1_set_opa_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_ready_cb(&a, unir_ui_fish_plus1_ready_cb);
    lv_anim_start(&a);
}

static void unir_ui_fish_tap(void)
{
    lv_timer_t *t;

    g_fish_count++;
    unir_ui_fish_spawn_plus1();
    unir_ui_fish_refresh_count(QOSA_TRUE);
    unir_ui_fish_refresh_img(QOSA_TRUE);

    t = lv_timer_create(unir_ui_fish_anim_revert_cb, FISH_ANIM_MS, QOSA_NULL);
    lv_timer_set_repeat_count(t, 1);
}

static void unir_ui_build_fish_page(void)
{
    p_fish = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_fish);
    lv_obj_add_flag(p_fish, LV_OBJ_FLAG_HIDDEN);

    img_fish = lv_img_create(p_fish);
    lv_img_set_src(img_fish, &img_muyu_bg);
    lv_obj_set_pos(img_fish, 0, 0); /* 木鱼图片全屏覆盖 (240x240) */
    lv_img_set_pivot(img_fish, img_muyu_bg.header.w / 2, img_muyu_bg.header.h / 2);

    lbl_fish_count = unir_ui_label_create(p_fish, FISH_COUNT_Y, &lv_font_montserrat_28, UI_LV_WHITE());
    lv_obj_set_style_radius(lbl_fish_count, 4, 0);

    lbl_fish_hint = unir_ui_label_create(p_fish, FISH_HINT_Y, QOSA_NULL, UI_LV_GRAY());

    unir_ui_fish_refresh_count(QOSA_FALSE);
    unir_ui_fish_refresh_img(QOSA_FALSE);
    unir_ui_fish_refresh_hints();
}

/* ---------------------------------------------------------------------------
 * 计算器（CALC 卡片）：顶部显示行 + 4x4 键盘(7 8 9 ÷ / 4 5 6 × / 1 2 3 − / C 0 = +)，
 * 支持连续四则运算；按钮本身在 LVGL 层面不可点击，命中判断和秒表 RESET 按钮一样，靠
 * CLICK/DCLICK 手势里的触摸坐标手动做矩形命中测试（见 unir_ui_handle_gesture）。
 * ------------------------------------------------------------------------- */
#define CALC_DISPLAY_Y       42  /* 显示行：当前输入的数字或上一步结果 */
#define CALC_DISPLAY_BUF_LEN 32

#define CALC_BTN_COLS         4
#define CALC_BTN_ROWS         4
#define CALC_BTN_W           36
#define CALC_BTN_H           34
#define CALC_BTN_GAP          4
#define CALC_GRID_W          (CALC_BTN_COLS * CALC_BTN_W + (CALC_BTN_COLS - 1) * CALC_BTN_GAP)
#define CALC_GRID_X          ((LCD_WIDTH - CALC_GRID_W) / 2)
#define CALC_GRID_TOP_Y      66  /* 键盘区起始 y：留出上方显示行空间，同时让最下一行尽量靠近圆屏
                                     竖直中点，减小四角被物理圆形边缘裁掉的风险（估算值，建议实机确认） */
#define CALC_HIT_MARGIN       2  /* 按钮间距较小，命中测试只做很小的外扩，避免相邻按钮抢触摸 */

static const char *const CALC_BTN_TEXT[CALC_BTN_ROWS * CALC_BTN_COLS] =
{
    "7", "8", "9", "/",
    "4", "5", "6", "*",
    "1", "2", "3", "-",
    "C", "0", "=", "+",
};

static lv_obj_t *p_calc;
static lv_obj_t *lbl_calc_display;
static lv_obj_t *obj_calc_btn[CALC_BTN_ROWS * CALC_BTN_COLS];
static lv_obj_t *lbl_calc_btn[CALC_BTN_ROWS * CALC_BTN_COLS];

static char        g_calc_display[CALC_DISPLAY_BUF_LEN] = "0"; /* 当前显示的数字：正在输入的操作数，或上一步/最终结果 */
static double      g_calc_operand1   = 0.0;      /* 保存的第一个操作数，按下运算符或 = 时固化 */
static char        g_calc_pending_op = 0;        /* 当前待执行的运算符 '+' '-' '*' '/'，0 表示没有 */
static qosa_bool_t g_calc_new_entry  = QOSA_TRUE; /* 为真时下一次按数字会清空显示重新开始输入，而不是追加 */
static qosa_bool_t g_calc_error      = QOSA_FALSE; /* 除以 0 等错误状态，需按 C 才能恢复 */

static void unir_ui_calc_refresh_display(void)
{
    lv_label_set_text(lbl_calc_display, g_calc_display);
}

/* 清空计算器所有状态，回到初始 "0"（对应 C 按钮） */
static void unir_ui_calc_reset(void)
{
    snprintf(g_calc_display, sizeof(g_calc_display), "0");
    g_calc_operand1   = 0.0;
    g_calc_pending_op = 0;
    g_calc_new_entry  = QOSA_TRUE;
    g_calc_error      = QOSA_FALSE;
    unir_ui_calc_refresh_display();
}

/* 依据 g_calc_pending_op 对 operand1 和当前显示值做运算，结果格式化写回 g_calc_display；
   除以 0 时切换到错误态，显示 "Error"，需要按 C 才能恢复 */
static void unir_ui_calc_apply_pending(void)
{
    double operand2 = atof(g_calc_display);
    double result    = 0.0;

    switch (g_calc_pending_op)
    {
        case '+': result = g_calc_operand1 + operand2; break;
        case '-': result = g_calc_operand1 - operand2; break;
        case '*': result = g_calc_operand1 * operand2; break;
        case '/':
            if (operand2 == 0.0)
            {
                snprintf(g_calc_display, sizeof(g_calc_display), "Error");
                g_calc_error = QOSA_TRUE;
                return;
            }
            result = g_calc_operand1 / operand2;
            break;
        default:
            return;
    }

    snprintf(g_calc_display, sizeof(g_calc_display), "%g", result); /* %g 保持结果简短，避免超出显示行宽度 */
    g_calc_operand1 = result;
}

static void unir_ui_calc_press_digit(char digit)
{
    size_t len;

    if (g_calc_error)
    {
        return;
    }

    if (g_calc_new_entry || strcmp(g_calc_display, "0") == 0)
    {
        g_calc_display[0] = digit;
        g_calc_display[1] = '\0';
        g_calc_new_entry  = QOSA_FALSE;
    }
    else
    {
        len = strlen(g_calc_display);
        if (len + 1 < sizeof(g_calc_display) - 1) /* 预留结尾符，限制最长输入位数，避免显示溢出 */
        {
            g_calc_display[len]     = digit;
            g_calc_display[len + 1] = '\0';
        }
    }

    unir_ui_calc_refresh_display();
}

static void unir_ui_calc_press_op(char op)
{
    if (g_calc_error)
    {
        return;
    }

    if (g_calc_pending_op != 0 && !g_calc_new_entry)
    {
        /* 已有未完成的运算且已经输入了新的第二操作数：先算出中间结果再继续下一次运算，支持连续四则运算 */
        unir_ui_calc_apply_pending();
        unir_ui_calc_refresh_display();
        if (g_calc_error)
        {
            return;
        }
    }
    else
    {
        g_calc_operand1 = atof(g_calc_display);
    }

    g_calc_pending_op = op;
    g_calc_new_entry  = QOSA_TRUE;
}

static void unir_ui_calc_press_equals(void)
{
    if (g_calc_error || g_calc_pending_op == 0)
    {
        return;
    }

    unir_ui_calc_apply_pending();
    unir_ui_calc_refresh_display();
    g_calc_pending_op = 0;
    g_calc_new_entry  = QOSA_TRUE;
}

/* 按下键盘上某个按钮(index 0..15)：按按钮文字内容分派到数字/运算符/等号/清零处理，
   并复用秒表 RESET 按钮的按下反馈动效(unir_ui_btn_flash) */
static void unir_ui_calc_press_btn(int index)
{
    const char *text = CALC_BTN_TEXT[index];

    unir_ui_btn_flash(obj_calc_btn[index]);

    if (text[0] == 'C')
    {
        unir_ui_calc_reset();
    }
    else if (text[0] == '=')
    {
        unir_ui_calc_press_equals();
    }
    else if (text[0] == '+' || text[0] == '-' || text[0] == '*' || text[0] == '/')
    {
        unir_ui_calc_press_op(text[0]);
    }
    else
    {
        unir_ui_calc_press_digit(text[0]);
    }
}

static void unir_ui_build_calc_page(void)
{
    int i;

    p_calc = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_calc);
    lv_obj_add_flag(p_calc, LV_OBJ_FLAG_HIDDEN);

    lbl_calc_display = unir_ui_label_create(p_calc, CALC_DISPLAY_Y, &lv_font_montserrat_20, UI_LV_WHITE());

    for (i = 0; i < CALC_BTN_ROWS * CALC_BTN_COLS; i++)
    {
        int row = i / CALC_BTN_COLS;
        int col = i % CALC_BTN_COLS;
        int x    = CALC_GRID_X     + col * (CALC_BTN_W + CALC_BTN_GAP);
        int y    = CALC_GRID_TOP_Y + row * (CALC_BTN_H + CALC_BTN_GAP);

        obj_calc_btn[i] = lv_btn_create(p_calc);
        lv_obj_set_size(obj_calc_btn[i], CALC_BTN_W, CALC_BTN_H);
        lv_obj_set_pos(obj_calc_btn[i], x, y);
        lv_obj_clear_flag(obj_calc_btn[i], LV_OBJ_FLAG_CLICKABLE); /* 和秒表 RESET 按钮一样，命中判断由触摸手势代码手动完成 */
        lv_obj_set_style_radius(obj_calc_btn[i], 6, 0);
        lv_obj_set_style_bg_color(obj_calc_btn[i], UI_LV_GRAY(), 0);

        lbl_calc_btn[i] = lv_label_create(obj_calc_btn[i]);
        lv_label_set_text(lbl_calc_btn[i], CALC_BTN_TEXT[i]);
        lv_obj_set_style_text_color(lbl_calc_btn[i], UI_LV_BLACK(), 0);
        lv_obj_center(lbl_calc_btn[i]);
    }

    /* "=" 按钮(索引14)用主题色区分，和真实计算器的确认键风格保持一致 */
    lv_obj_set_style_bg_color(obj_calc_btn[14], lv_color_hex(0x4A90D9), 0);
    lv_obj_set_style_text_color(lbl_calc_btn[14], UI_LV_WHITE(), 0);

    unir_ui_calc_reset();
}

/* ---------------- 功能选择页：图标垂直排列，上下滑动切换选中项 ----------------
 * 选中项的图标放大并居中显示在屏幕纵向中点，名称固定显示在图标右侧；上下各最多再显示
 * 两行(共 5 行)，越远离选中项的行图标越小、越靠内侧，避免被圆屏边缘物理裁掉；距离选中项
 * 超过两行的项才隐藏。 */
#define FEAT_ICON_SIZE      64  /* 图标源图尺寸，需与 tools/gen_feature_icons.py 生成的图标像素尺寸一致 */
#define FEAT_MAX_ROW_DIST    2  /* 上下各最多显示的相邻行数，超过这个距离的项隐藏 */
#define FEAT_TOP_SWIPE_Y_MAX 80 /* 屏幕顶部区域的 y 坐标上限：在这个区域内下滑时不管当前选中第几项，
                                    都直接返回表盘页，不用先退回第一项再继续下滑（估算值，建议实机确认） */

/* 按“距选中项的行距离”(0=选中行,1=相邻行,2=再远一行)索引的布局参数表：行距越大，
   圆屏在该高度的可视宽度越窄，所以图标越小、横坐标和名称整体越靠内侧，避免被圆形边缘裁掉 */
static const int FEAT_ROW_CY_OFS[FEAT_MAX_ROW_DIST + 1] = { 0,  46,  80 }; /* 相对屏幕纵向中点的垂直偏移 */
static const int FEAT_ICON_SZ[FEAT_MAX_ROW_DIST + 1]    = { 72, 42,  24 }; /* 图标显示直径 */
static const int FEAT_ICON_CX[FEAT_MAX_ROW_DIST + 1]    = { 50, 54,  60 }; /* 图标圆心横坐标：相邻行比选中行略右移，
                                                                               再远一行比相邻行继续多右移一点，贴合圆屏边框弧度 */
static const int FEAT_LABEL_X[FEAT_MAX_ROW_DIST + 1]    = { 96, 100, 106 }; /* 名称标签左边缘横坐标：与图标同步右移，
                                                                                增量和 FEAT_ICON_CX 保持一致 */
static const int FEAT_LABEL_W[FEAT_MAX_ROW_DIST + 1]    = { 108, 90,  78 }; /* 名称标签宽度：行距越大越窄，
                                                                               配合右移后仍不超出圆屏可视范围 */
static const lv_font_t *const FEAT_LABEL_FONT[FEAT_MAX_ROW_DIST + 1] =
{
    &lv_font_montserrat_16, &lv_font_montserrat_14, &lv_font_montserrat_14
}; /* 名称字体：选中行用大一号(16px)，其余行用 14px——lv_conf.h 只启用了
            montserrat 14/16/20/28/40，14px 已是当前编译配置中最小的内置字体，故 dist1/dist2 相同 */
static const int FEAT_LABEL_LINE_H[FEAT_MAX_ROW_DIST + 1] = { 20, 18, 18 }; /* 对应字体的单行文字估算行高，用于垂直居中 */

static lv_obj_t *p_feature;
static lv_obj_t *frame_feat_icon[UI_FEATURE_COUNT];/* 与图标等大的圆形裁剪框，把方形图片裁成圆形显示 */
static lv_obj_t *img_feat_icon[UI_FEATURE_COUNT];  /* 徽标里居中显示的图标图片 */
static lv_obj_t *lbl_feat_title[UI_FEATURE_COUNT]; /* 图标右侧的功能名称 */
static int        g_feat_sel_index = 0;             /* 当前选中项下标，0..UI_FEATURE_COUNT-1 */

/* 按 g_feat_sel_index 重新计算每一项的位置/缩放/显隐，构建时和每次上下滑动切换选中项后都会调用 */
static void unir_ui_feature_layout(void)
{
    int i;

    for (i = 0; i < UI_FEATURE_COUNT; i++)
    {
        int offset = i - g_feat_sel_index; /* 相对选中项的行偏移：0=选中(居中放大)，正数在下方，负数在上方 */
        int dist   = (offset < 0) ? -offset : offset;

        if (dist > FEAT_MAX_ROW_DIST)
        {
            /* 距离选中项超过两行的图标/名称不显示，避免圆屏空间拥挤或被物理裁掉 */
            lv_obj_add_flag(frame_feat_icon[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl_feat_title[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            int         is_sel  = (offset == 0);
            int         sign    = (offset > 0) ? 1 : ((offset < 0) ? -1 : 0);
            int         row_cy  = (LCD_HEIGHT / 2) + sign * FEAT_ROW_CY_OFS[dist];
            int         icon_sz = FEAT_ICON_SZ[dist];
            int         zoom    = icon_sz * 256 / FEAT_ICON_SIZE;
            const char *text    = s_features[i].line1;
            int         lines   = 1;
            char        buf[32];

            lv_obj_clear_flag(frame_feat_icon[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lbl_feat_title[i], LV_OBJ_FLAG_HIDDEN);

            lv_obj_set_size(frame_feat_icon[i], icon_sz, icon_sz);
            lv_obj_set_pos(frame_feat_icon[i], FEAT_ICON_CX[dist] - icon_sz / 2, row_cy - icon_sz / 2);

            /* 相邻行右移后可能与选中项的图标区域有重叠，LVGL 按创建顺序/最近操作顺序决定遮盖关系，
               这里强制把选中项图标提到最顶层，确保它始终显示在其他行图标之上，不被遮挡 */
            if (is_sel)
            {
                lv_obj_move_foreground(frame_feat_icon[i]);
            }

            lv_img_set_zoom(img_feat_icon[i], zoom);
            lv_obj_set_pos(img_feat_icon[i], icon_sz / 2 - FEAT_ICON_SIZE / 2, icon_sz / 2 - FEAT_ICON_SIZE / 2);

            /* 只有选中项才展开第二行副标题(如 FISH/COUNTER)，其余行只显示主标题，
               避免两行文字在圆屏边缘被物理裁掉 */
            if (is_sel && s_features[i].line2 != QOSA_NULL)
            {
                snprintf(buf, sizeof(buf), "%s\n%s", s_features[i].line1, s_features[i].line2);
                text  = buf;
                lines = 2;
            }
            lv_label_set_text(lbl_feat_title[i], text);
            lv_obj_set_style_text_font(lbl_feat_title[i], FEAT_LABEL_FONT[dist], 0);
            lv_obj_set_width(lbl_feat_title[i], FEAT_LABEL_W[dist]);
            lv_obj_set_pos(lbl_feat_title[i], FEAT_LABEL_X[dist], row_cy - (lines * FEAT_LABEL_LINE_H[dist]) / 2);
        }
    }
}

static void unir_ui_build_feature_page(void)
{
    int i;

    p_feature = lv_obj_create(lv_scr_act());
    unir_ui_page_container_init(p_feature);
    lv_obj_add_flag(p_feature, LV_OBJ_FLAG_HIDDEN);

    /* 这里只负责创建控件本身，具体位置/缩放/显隐统一交给 unir_ui_feature_layout() 计算 */
    for (i = 0; i < UI_FEATURE_COUNT; i++)
    {
        const ui_feature_t *feat   = &s_features[i];
        lv_color_t          accent = lv_color_hex(feat->accent_hex);

        /* 圆形裁剪框：圆角半径设为 LV_RADIUS_CIRCLE 并开启 clip_corner，子对象(方形图标图片)
           超出圆形区域的部分会被裁掉，从而呈现圆形图标效果；尺寸/位置随选中状态动态变化 */
        frame_feat_icon[i] = lv_obj_create(p_feature);
        lv_obj_remove_style(frame_feat_icon[i], QOSA_NULL, LV_PART_MAIN | LV_STATE_ANY);
        if (feat->kind == UI_FEATURE_TOMATO)
        {
            /* 番茄钟图标自身四周是浅黄/米白底色，圆形裁剪框背景同色填充，消除抗锯齿边缘的黑色描边 */
            lv_obj_set_style_bg_opa(frame_feat_icon[i], LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(frame_feat_icon[i], lv_color_hex(FEAT_TOMATO_BADGE_BG_HEX), 0);
        }
        else if (feat->kind == UI_FEATURE_CALC)
        {
            /* 计算器图标自身四周是白色圆角底色，圆形裁剪框背景同色填充，道理同番茄钟 */
            lv_obj_set_style_bg_opa(frame_feat_icon[i], LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(frame_feat_icon[i], lv_color_hex(FEAT_CALC_BADGE_BG_HEX), 0);
        }
        else
        {
            lv_obj_set_style_bg_opa(frame_feat_icon[i], LV_OPA_TRANSP, 0);
        }
        lv_obj_set_style_border_width(frame_feat_icon[i], 0, 0);
        lv_obj_set_style_pad_all(frame_feat_icon[i], 0, 0);
        lv_obj_set_style_radius(frame_feat_icon[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_clip_corner(frame_feat_icon[i], true, 0);
        lv_obj_clear_flag(frame_feat_icon[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(frame_feat_icon[i], LV_OBJ_FLAG_CLICKABLE);

        img_feat_icon[i] = lv_img_create(frame_feat_icon[i]);
        lv_img_set_src(img_feat_icon[i], feat->icon);
        lv_img_set_pivot(img_feat_icon[i], FEAT_ICON_SIZE / 2, FEAT_ICON_SIZE / 2);
        lv_obj_clear_flag(img_feat_icon[i], LV_OBJ_FLAG_CLICKABLE);

        lbl_feat_title[i] = lv_label_create(p_feature);
        lv_obj_set_style_text_align(lbl_feat_title[i], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_text_color(lbl_feat_title[i], accent, 0);
    }

    unir_ui_feature_layout();
}

/* 构建所有页面控件（只在启动时调用一次），再把除表盘外的页面都隐藏起来 */
static void unir_ui_build(void)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_set_style_bg_color(scr, UI_LV_BLACK(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    unir_ui_build_clock_page();
    unir_ui_build_feature_page();
    unir_ui_build_stopwatch_page();
    unir_ui_build_tomato_page();
    unir_ui_build_fish_page();
    unir_ui_build_calc_page();
}

/* 切换到指定页面：隐藏其余页面容器，显示目标页面，并刷新一次目标页面的动态内容 */
static void unir_ui_show_page(ui_page_e page)
{
    lv_obj_add_flag(p_clock,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_feature, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_sw,      LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_tomato,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_fish,    LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p_calc,    LV_OBJ_FLAG_HIDDEN);

    g_ui_page = page;

    switch (page)
    {
        case UI_PAGE_CLOCK:
            lv_obj_clear_flag(p_clock, LV_OBJ_FLAG_HIDDEN);
            unir_ui_refresh_clock(QOSA_NULL);
            break;

        case UI_PAGE_FEATURE:
            lv_obj_clear_flag(p_feature, LV_OBJ_FLAG_HIDDEN);
            unir_ui_feature_layout();
            break;

        case UI_PAGE_STOPWATCH_RUN:
            lv_obj_clear_flag(p_sw, LV_OBJ_FLAG_HIDDEN);
            break;

        case UI_PAGE_TOMATO_RUN:
            lv_obj_clear_flag(p_tomato, LV_OBJ_FLAG_HIDDEN);
            unir_ui_tomato_refresh();
            break;

        case UI_PAGE_FISH_RUN:
            lv_obj_clear_flag(p_fish, LV_OBJ_FLAG_HIDDEN);
            break;

        case UI_PAGE_CALC_RUN:
            lv_obj_clear_flag(p_calc, LV_OBJ_FLAG_HIDDEN);
            break;
    }
}

/* 按当前页面 + 手势类型决定页面跳转/功能切换 */
static void unir_ui_handle_gesture(uint8_t gesture)
{
    switch (g_ui_page)
    {
        case UI_PAGE_CLOCK:
            if (gesture == TP_GESTURE_UP)
            {
                unir_ui_show_page(UI_PAGE_FEATURE);
            }
            break;

        case UI_PAGE_FEATURE:
            if (gesture == TP_GESTURE_DOWN)
            {
                /* 在屏幕顶部区域下滑时不管当前选中哪一项，都直接返回表盘页（和表盘页上滑进入功能选择页
                   的手势区域对应），无需先退回第一项再继续下滑；其余位置下滑仍按原来逻辑处理 */
                if (g_tp_touch_y != 0xFFFF && g_tp_touch_y <= FEAT_TOP_SWIPE_Y_MAX)
                {
                    unir_ui_show_page(UI_PAGE_CLOCK);
                }
                /* 已经在第一项时继续下滑才返回表盘页，否则表示切换到上一项 */
                else if (g_feat_sel_index > 0)
                {
                    g_feat_sel_index--;
                    unir_ui_feature_layout();
                }
                else
                {
                    unir_ui_show_page(UI_PAGE_CLOCK);
                }
            }
            else if (gesture == TP_GESTURE_UP)
            {
                /* 已经在最后一项时上滑无效，否则切换到下一项 */
                if (g_feat_sel_index < UI_FEATURE_COUNT - 1)
                {
                    g_feat_sel_index++;
                    unir_ui_feature_layout();
                }
            }
            else if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK)
            {
                if (s_features[g_feat_sel_index].kind == UI_FEATURE_STOPWATCH)
                {
                    unir_ui_show_page(UI_PAGE_STOPWATCH_RUN);
                }
                else if (s_features[g_feat_sel_index].kind == UI_FEATURE_TOMATO)
                {
                    unir_ui_show_page(UI_PAGE_TOMATO_RUN);
                }
                else if (s_features[g_feat_sel_index].kind == UI_FEATURE_FISH)
                {
                    unir_ui_show_page(UI_PAGE_FISH_RUN);
                }
                else if (s_features[g_feat_sel_index].kind == UI_FEATURE_CALC)
                {
                    unir_ui_show_page(UI_PAGE_CALC_RUN);
                }
            }
            break;

        case UI_PAGE_STOPWATCH_RUN:
            if (gesture == TP_GESTURE_DOWN)
            {
                unir_ui_show_page(UI_PAGE_FEATURE);
            }
            else if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK)
            {
                if (g_tp_touch_x != 0xFFFF && unir_ui_point_in_obj(obj_sw_reset_btn, SW_HIT_MARGIN, g_tp_touch_x, g_tp_touch_y))
                {
                    unir_ui_btn_flash(obj_sw_reset_btn);
                    unir_ui_sw_reset();
                }
                else
                {
                    unir_ui_sw_set_running((qosa_bool_t)!g_sw_running);
                }
            }
            else if (gesture == TP_GESTURE_LONGPRESS)
            {
                unir_ui_sw_reset();
            }
            break;

        case UI_PAGE_TOMATO_RUN:
            if (gesture == TP_GESTURE_DOWN)
            {
                /* 完成页(DONE)下滑：直接复位并留在番茄钟页面重新开始倒计时，不用先退回
                   功能选择页再点一次进来；未完成(运行中/暂停中)下滑则按原来的逻辑返回
                   功能选择页。 */
                if (g_tomato_done)
                {
                    unir_ui_tomato_reset();
                }
                else
                {
                    unir_ui_show_page(UI_PAGE_FEATURE);
                }
            }
            else if (gesture == TP_GESTURE_LEFT)
            {
                /* 左滑：减少 1 分钟时长（仅在未开始计时时生效） */
                unir_ui_tomato_adjust(-1);
            }
            else if (gesture == TP_GESTURE_RIGHT)
            {
                /* 右滑：增加 1 分钟时长（仅在未开始计时时生效） */
                unir_ui_tomato_adjust(1);
            }
            else if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK)
            {
                /* 倒计时跑完后单击/双击不再重新开始，必须长按复位 */
                if (!g_tomato_done)
                {
                    unir_ui_tomato_set_running((qosa_bool_t)!g_tomato_running);
                }
            }
            else if (gesture == TP_GESTURE_LONGPRESS)
            {
                /* 长按复位：不论当前是运行中、暂停中还是已完成，都能直接复位，
                   和秒表的长按复位手势保持一致（已验证可靠），不再依赖双击 */
                unir_ui_tomato_reset();
            }
            break;

        case UI_PAGE_FISH_RUN:
            if (gesture == TP_GESTURE_DOWN)
            {
                if (g_fish_confirm_pending)
                {
                    /* 清零确认状态下下滑表示取消确认，留在木鱼页继续计数 */
                    g_fish_confirm_pending = QOSA_FALSE;
                    unir_ui_fish_refresh_hints();
                }
                else
                {
                    unir_ui_show_page(UI_PAGE_FEATURE);
                }
            }
            else if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK)
            {
                if (g_fish_confirm_pending)
                {
                    /* 再点一次确认清零 */
                    g_fish_count           = 0;
                    g_fish_confirm_pending = QOSA_FALSE;
                    unir_ui_fish_refresh_count(QOSA_FALSE);
                    unir_ui_fish_refresh_hints();
                }
                else
                {
                    unir_ui_fish_tap();
                }
            }
            else if (gesture == TP_GESTURE_LONGPRESS)
            {
                /* 长按进入清零确认状态，不直接清零，必须再点一次才真正清零 */
                if (!g_fish_confirm_pending)
                {
                    g_fish_confirm_pending = QOSA_TRUE;
                    unir_ui_fish_refresh_hints();
                }
            }
            break;

        case UI_PAGE_CALC_RUN:
            if (gesture == TP_GESTURE_DOWN)
            {
                unir_ui_show_page(UI_PAGE_FEATURE);
            }
            else if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK)
            {
                int i;

                if (g_tp_touch_x != 0xFFFF)
                {
                    for (i = 0; i < CALC_BTN_ROWS * CALC_BTN_COLS; i++)
                    {
                        if (unir_ui_point_in_obj(obj_calc_btn[i], CALC_HIT_MARGIN, g_tp_touch_x, g_tp_touch_y))
                        {
                            unir_ui_calc_press_btn(i);
                            break;
                        }
                    }
                }
            }
            break;
    }
}

/* 在任务循环里被周期调用：有中断标志才去读 I2C，没有触摸时开销接近 0。
   冷却期用来防止"手指持续接触屏幕期间同一个手势被连续重复上报"：滑动(上下左右)在物理滑动
   过程中会这样，长按(LONGPRESS)在手指按住不放期间同样会每个扫描周期重复上报一次，导致长按
   复位被连续触发多次（实测现象：长按一次相当于连续复位好几次）。点击/双击是瞬间离散的触摸
   释放事件，不会重复上报，且秒表要求支持快速连续点击两次切换开始/停止，所以只有这两种手势
   不受冷却限制，直接处理。 */
static void unir_tp_poll(void)
{
    uint8_t gesture = TP_GESTURE_NONE;

    if (g_tp_cooldown_ms > 0)
    {
        g_tp_cooldown_ms = (g_tp_cooldown_ms > UI_POLL_INTERVAL_MS) ? (g_tp_cooldown_ms - UI_POLL_INTERVAL_MS) : 0;
    }

    /* 不论当前有没有触摸事件都要周期性保活，防止芯片在长时间无操作后自行休眠 */
    g_tp_keepalive_ms += UI_POLL_INTERVAL_MS;
    if (g_tp_keepalive_ms >= TP_KEEPALIVE_MS)
    {
        g_tp_keepalive_ms = 0;
        (void)unir_tp_write_reg(TP_REG_DIS_SLEEP, 0x01, 1);
    }

    if (!g_tp_irq_flag)
    {
        return;
    }
    g_tp_irq_flag = QOSA_FALSE;

    if (unir_tp_read_reg(TP_REG_GESTURE, &gesture, 1) != QOSA_I2C_SUCCESS || gesture == TP_GESTURE_NONE)
    {
        return;
    }

    if (gesture == TP_GESTURE_UP || gesture == TP_GESTURE_DOWN ||
        gesture == TP_GESTURE_LEFT || gesture == TP_GESTURE_RIGHT ||
        gesture == TP_GESTURE_LONGPRESS)
    {
        if (g_tp_cooldown_ms > 0)
        {
            return;
        }
        g_tp_cooldown_ms = TP_GESTURE_COOLDOWN_MS;
    }

    g_tp_touch_x = 0xFFFF;
    if (gesture == TP_GESTURE_CLICK || gesture == TP_GESTURE_DCLICK || gesture == TP_GESTURE_DOWN)
    {
        /* 点击/双击读取触点坐标用来判断是否点在 RESET 按钮上；下滑(DOWN)读取触点坐标用来判断
           是否在屏幕顶部区域滑动(功能选择页用于支持"顶部下滑直接返回表盘页")；读取失败就按 0xFFFF 处理 */
        (void)unir_tp_read_point(&g_tp_touch_x, &g_tp_touch_y);
    }

    unir_ui_handle_gesture(gesture);
}

/* 常驻任务：每 50ms 轮询一次触摸事件，同时驱动 LVGL 的 tick/timer（表盘/秒表/番茄钟的
   周期性刷新都已经改成各自的 lv_timer，不需要在这里手动调用） */
static void small_screen_task(void *argv)
{
    (void)argv;

    unir_ui_show_page(UI_PAGE_CLOCK);

    for (;;)
    {
        unir_tp_poll();
        lv_tick_inc(UI_POLL_INTERVAL_MS);
        lv_timer_handler();
        qosa_task_sleep_ms(UI_POLL_INTERVAL_MS);
    }
}

/* 应用入口：点亮屏幕、初始化 LVGL 并搭好所有页面控件、初始化触摸后拉起常驻任务 */
static void small_screen_init(void)
{
    qosa_task_t task_ref = QOSA_NULL;

    qosa_gpio_set_voltage(VOL_3_30V);
    unir_spilcd_init();

    lv_init();
    unir_lvgl_disp_init();
    unir_ui_build();

    /* 开机时读取上次保存的 NV/RTC 时间作为初始值（而不是默认回到 1970 年），
       并在模组注册到网络后自动用基站时间同步校准，保持我们手动设置的时区不被网络覆盖 */
    {
        qosa_rtc_cfg_t rtc_cfg = {0};

        rtc_cfg.nv_cfg  = QOSA_RTC_CFG_ENABLE;
        rtc_cfg.rtc_cfg = QOSA_RTC_CFG_ENABLE;
        rtc_cfg.nwt_cfg = QOSA_RTC_CFG_ENABLE;
        rtc_cfg.tz_cfg  = QOSA_RTC_CFG_TZ_NO_UPDATE;

        if (qosa_rtc_set_cfg(&rtc_cfg) != QOSA_RTC_ERR_OK)
        {
            QLOGE("rtc set cfg failed");
        }
    }

    if (qosa_rtc_set_timezone(RTC_TIMEZONE_QUARTER_HOURS) != 0)
    {
        QLOGE("rtc set timezone failed");
    }

    if (unir_tp_init() != 0)
    {
        QLOGE("touch panel init failed, page navigation via touch will not work");
    }

    /* LVGL 渲染(字体解码/对象树遍历)比原来的手绘字体占用更多栈空间，栈从 4096 提到 12288，
       与 unirtos-lvgl-demos-1.0.0 参考例程里验证过的栈大小保持一致 */
    if (qosa_task_create(&task_ref, 12288, QOSA_PRIORITY_NORMAL, "small_screen", small_screen_task, QOSA_NULL) != 0)
    {
        QLOGE("small_screen task create failed");
    }
}

UNIRTOS_APP_EXPORT(700, "small_screen_demo", small_screen_init);

