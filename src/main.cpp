#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <algorithm>
#include <String.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nes.h"
#include "lgfx_conf.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_timer.h"

// 串口调试开关
#ifndef ENABLE_DEBUG_SERIAL
#define ENABLE_DEBUG_SERIAL false
#endif

// ================ 菜单颜色配置 (高级灰色调) ================
#define MENU_BG_COLOR       0x2104  // 深灰背景 (RGB: 32, 32, 32)
#define MENU_HEADER_COLOR   0x4A69  // 中灰标题背景 (RGB: 72, 77, 72)
#define MENU_TEXT_COLOR     0xC618  // 浅灰文字 (RGB: 192, 192, 192)
#define MENU_HIGHLIGHT_BG   0xFDE0  // 选中项背景 (RGB: 255, 255, 0)
#define MENU_ARROW_COLOR    0xAD75  // 箭头颜色 (RGB: 168, 174, 168)
#define MENU_HINT_COLOR     0x7BCF  // 提示文字颜色 (RGB: 120, 120, 120)
#define MENU_TITLE_COLOR    0xE71C  // 标题文字 (RGB: 224, 224, 224)
#define MENU_BORDER_COLOR   0x52AA  // 边框颜色 (RGB: 80, 85, 80)
#define PAUSE_OVERLAY_COLOR 0x18C3  // 暂停遮罩 (深色半透明效果)

// ================ 菜单状态 ================
enum AppState {
    STATE_MENU,     // 主菜单
    STATE_PLAYING,  // 游戏中
    STATE_PAUSED    // 暂停菜单
};

static AppState currentState = STATE_MENU;
static std::vector<String> romList;       // ROM 文件列表
static int selectedIndex = 0;             // 当前选中的游戏索引
static int scrollOffset = 0;              // 滚动偏移
static const int ITEMS_PER_PAGE = 10;      // 每页显示的游戏数量
static int pauseMenuIndex = 0;            // 暂停菜单选项索引

// 按键防抖
static unsigned long lastButtonTime = 0;
static const unsigned long BUTTON_DEBOUNCE = 200;  // 200ms防抖

#if ENABLE_DEBUG_SERIAL
#define FPS_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define FPS_PRINT(...) ((void)0)
#endif

// ================ PIN定义 ================
// SD卡引脚
#define SD_CS_PIN     14//42
#define SD_SCLK_PIN   13//40
#define SD_MISO_PIN   11//方型为8，长方型为18//39
#define SD_MOSI_PIN   12//方型为18，长方型为8//41
#define SD_FREQ       10000000  // 10 MHz

// 游戏控制器按键
#define A_BUTTON      6//17
#define B_BUTTON      4//18
#define X_BUTTON      7//17
#define Y_BUTTON      5//18

// 连发配置
#define TURBO_INTERVAL_MS  60    // 连发间隔（毫秒），可调整

//#define LEFT_BUTTON   8
//#define RIGHT_BUTTON  18
//#define UP_BUTTON     17
//#define DOWN_BUTTON   3
#define START_BUTTON  3//15
#define SELECT_BUTTON 8//16

// I2S / APU -> MAX98357A (I2S DAC)
#define I2S_BCLK_PIN 16//5
#define I2S_LRCLK_PIN 17//4
#define I2S_DATA_PIN 15//6

// 新增摇杆 ADC 引脚   追加代码
#define JOYSTICK_X_PIN    9//1   // 摇杆 X 轴（左右）
#define JOYSTICK_Y_PIN   10//2   // 摇杆 Y 轴（上下）


// 死区阈值（ADC 12位，范围 0~4095，中间值约 1500）   追加代码
#define JOYSTICK_DEADZONE  500   // 死区范围 ±800
//#define JOYSTICK_CENTER    2048  // 中心值
int JOYSTICK_CENTER_X=1800;   // 从 2048 改为 1500
int JOYSTICK_CENTER_Y=1800;  // 从 2048 改为 1500

// 音频参数
constexpr int AUDIO_SAMPLE_RATE = 44100;
constexpr int I2S_NUM = 0;

// ================ 全局变量 ================
NES nes;
LGFX tft;


// 屏幕参数
constexpr int SCREEN_WIDTH  = 256;
constexpr int SCREEN_HEIGHT = 240;
constexpr int DISPLAY_WIDTH = 480;      // 目标显示宽度
constexpr int DISPLAY_HEIGHT = 320;     // 目标显示高度
constexpr int TFT_OFFSET_X  = 0; // 横向居中
constexpr int OVERSCAN_CROP_X = 4;  // 隐藏 LCD 上可见的横向卷轴边缘接缝
//constexpr int DISPLAY_WIDTH = SCREEN_WIDTH;
// 每个块的行数（用 DMA 一次推多行以减少 setAddrWindow/wait 开销）
// 8 行 = 30 次 DMA/帧，60 行 = 4 次 DMA/帧，120 行 = 2 次 DMA/帧
constexpr int BLOCK_LINES = 60;  // 增大到 60 行，240/60=4 次 DMA 每帧
constexpr int DISPLAY_BLOCK_LINES = 16;

// FPS 统计变量
static uint32_t last_emulation_us = 0;  // 最近一次仿真帧耗时（微秒）
static uint32_t fps_count = 0;          // 已完成的仿真帧计数
static uint32_t fps_last_ms = 0;        // 上次打印 FPS 的时间戳
static uint32_t last_dma_us = 0;        // DMA 传输耗时
static uint32_t game_start_ms = 0;      // 当前游戏启动时间，用于启动失败保护
static uint32_t last_rendered_ms = 0;   // 最近一次成功入队渲染帧的时间

// Separate SPI bus for SD so it cannot reconfigure/conflict with the TFT SPI bus.
// If your TFT_eSPI setup uses HSPI, keep SD on FSPI.
SPIClass sdSPI(FSPI);

// 双缓冲：用于无撕裂推屏
static uint16_t* frame_buf[2] = {nullptr, nullptr};
static uint16_t* display_crop_buf = nullptr;
static volatile uint8_t render_buf_idx = 0;
// 记录最后一次被显示的缓冲索引（用于在跳帧时复用上一帧以避免闪烁）
static volatile uint8_t last_displayed_idx = 0;

static QueueHandle_t frame_queue = nullptr;

static void initializeAudio();
static void apu_task(void* arg);
static void muteAudio();

static bool gameJustEntered = false;
// 游戏暂停状态 - APU 任务使用
static volatile bool gameRunning = false;
static bool sdCardAvailable = false;  // SD 卡是否可用

// 帧同步
const uint32_t FRAME_TIME_US = 16667;  // ~60 FPS (1000000 / 60)
const int CPU_CYCLES_PER_FRAME = 29780; // NES: 1.79MHz / 60fps ≈ 29780 cycles

// 抽帧开关: true=启用抽帧(性能优先), false=每帧都渲染(画面优先)
// SMB1 等游戏会用隔帧闪烁表现受伤/无敌，使用奇数周期跳帧避免锁相。
static bool ENABLE_FRAMESKIP = true;
static uint64_t next_frame_us = 0;
static uint8_t force_render_frames = 0;
static uint8_t consecutive_skipped_frames = 0;
static uint8_t frameskip_phase = 0;

struct ButtonState {
    uint8_t A = 0;
    uint8_t B = 0;
    uint8_t LEFT = 0;
    uint8_t RIGHT = 0;
    uint8_t UP = 0;
    uint8_t DOWN = 0;
    uint8_t START = 0;
    uint8_t SELECT = 0;
} buttons;

// 连发状态管理
struct TurboState {
    unsigned long lastTickX;     // X键计时
    unsigned long lastTickY;     // Y键计时
    bool currentStateX;          // X键当前状态
    bool currentStateY;          // Y键当前状态
} turbo = {
    .lastTickX = 0,
    .lastTickY = 0,
    .currentStateX = false,
    .currentStateY = false
};

// ================ 函数前向声明 ================
void updateButtons();
void runFrame();
void scanROMFiles();
void drawMainMenu();
void drawMenuList();
void drawPauseMenu();
void handleMenuInput();
void handlePauseInput();
bool loadSelectedROM();
void returnToMainMenu();
void clearScreenForGame();
bool tryInitSD();  // 尝试初始化 SD 卡

static void resetFrameScheduler(uint8_t forceRenderFrames = 2) {
    next_frame_us = 0;
    force_render_frames = forceRenderFrames;
    consecutive_skipped_frames = 0;
    frameskip_phase = 0;
    nes.requestFrameSkip(false);
}

// 从 ROM 路径生成 Save State 路径 (将 .nes 替换为 .sav)
static void getSaveStatePath(char* savePath, size_t maxLen) {
    const char* romPath = nes.getCurrentRomPath();
    strncpy(savePath, romPath, maxLen - 1);
    savePath[maxLen - 1] = '\0';
    
    // 查找最后一个 '.' 并替换扩展名
    char* dot = strrchr(savePath, '.');
    if (dot && (dot - savePath + 5) < (int)maxLen) {
        strcpy(dot, ".sav");
    } else {
        // 没有扩展名，直接追加
        strncat(savePath, ".sav", maxLen - strlen(savePath) - 1);
    }
}

// ================ 初始化函数 ================
void initializeSerial() {
#if ENABLE_DEBUG_SERIAL
    Serial.begin(115200);
    delay(500);
    
#else
    (void)0;
#endif
}

void initializeScreen() {
    tft.init();
    //tft.setRotation(1);  // 横屏模式
    
    // 测试屏幕
    tft.fillScreen(TFT_BLACK);
    tft.drawRect(0, 0, tft.width()-1, tft.height()-1, TFT_WHITE);
    delay(2000);
    tft.fillScreen(TFT_BLACK);
    
    // 分配双缓冲（NES 原始分辨率）
    for (int i = 0; i < 2; i++) {
        frame_buf[i] = (uint16_t*)heap_caps_malloc(
            SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t),
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
        );
        if (frame_buf[i]) {
            memset(frame_buf[i], 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
        }
    }
    
    // 可选：保留 display_crop_buf 用于其他用途，但不再用于裁剪
    display_crop_buf = (uint16_t*)heap_caps_malloc(
        480 * 16 * sizeof(uint16_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
    );
}

// 在 loop() 中调用：检查帧完成并入队
static bool tryEnqueueFrame() {
    PPU& ppu = nes.getPPU();
    if (!ppu.frameReady) return false;

    // 如果本帧没有渲染（跳帧），不入队，直接清除标志
    if (!ppu.renderedThisFrame) {
        ppu.frameReady = false;
        return false;
    }

    uint8_t send_idx = render_buf_idx;
    // 如果本帧有实际渲染，发送当前渲染缓冲并切换到另一个用于下一帧渲染
    if (xQueueSend(frame_queue, &send_idx, 0) == pdTRUE) {
        // 切换到另一缓冲用于下一帧渲染
        render_buf_idx = 1 - render_buf_idx;
        ppu.frameBuffer = frame_buf[render_buf_idx];
        ppu.frameReady = false;
        last_rendered_ms = millis();
        return true;
    }
    return false;
}

// display_task：只负责 DMA 推送已渲染的帧缓冲
static void display_task(void* arg) {
    uint8_t buf_idx;
    
    const int TARGET_W = 480;
    const int TARGET_H = 320;
    
    // 预计算水平映射表（256 -> 480）
    static uint16_t x_map[480];
    static bool map_initialized = false;
    if (!map_initialized) {
        for (int x = 0; x < TARGET_W; x++) {
            x_map[x] = (x * SCREEN_WIDTH) / TARGET_W;  //原屏幕旋转自动拉伸，可能是320自动拉伸到480，与256进行还原到全屏
        }
        map_initialized = true;
    }
    
    // 预计算垂直映射表（240 -> 320）
    static uint16_t y_map[320];
    static bool y_map_initialized = false;
    if (!y_map_initialized) {
        for (int y = 0; y < TARGET_H; y++) {
            y_map[y] = (y * SCREEN_HEIGHT) / TARGET_H;
        }
        y_map_initialized = true;
    }
    
    // 分配行缓冲区
    uint16_t* lineBuffer = (uint16_t*)heap_caps_malloc(TARGET_W * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!lineBuffer) {
        static uint16_t staticBuffer[480];
        lineBuffer = staticBuffer;
    }
    
    for (;;) {
        if (xQueueReceive(frame_queue, &buf_idx, portMAX_DELAY) != pdTRUE)
            continue;
        
        uint32_t t0 = micros();
        uint16_t* buf = frame_buf[buf_idx];
        
        tft.startWrite();
        
        for (int y = 0; y < TARGET_H; y++) {
            uint16_t* srcLine = buf + y_map[y] * SCREEN_WIDTH;
            
            for (int x = 0; x < TARGET_W; x++) {
                lineBuffer[x] = srcLine[x_map[x]];
            }
            
            tft.setAddrWindow(0, y, TARGET_W, 1);
            tft.pushPixelsDMA(lineBuffer, TARGET_W);
            tft.waitDMA();
        }
        
        tft.endWrite();
        last_displayed_idx = buf_idx;
        last_dma_us = micros() - t0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 行级帧调度（方法1使用）
void runFrame() {
    const int SCANLINES_PER_FRAME = 262;
    for (int i = 0; i < SCANLINES_PER_FRAME; ++i) {
        nes.stepScanline();
    }
}

void initializeButtons() {
    // 需要根据实际硬件布局调整这些引脚
    pinMode(A_BUTTON, INPUT_PULLUP);
    pinMode(B_BUTTON, INPUT_PULLUP);
    pinMode(X_BUTTON, INPUT_PULLUP);// X键作为连发A
    pinMode(Y_BUTTON, INPUT_PULLUP);// Y键作为连发B
    //pinMode(LEFT_BUTTON, INPUT_PULLUP);
    //pinMode(RIGHT_BUTTON, INPUT_PULLUP);
    //pinMode(UP_BUTTON, INPUT_PULLUP);
    //pinMode(DOWN_BUTTON, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(SELECT_BUTTON, INPUT_PULLUP);

    // 初始化连发状态 - 分别初始化
    turbo.lastTickX = 0;
    turbo.lastTickY = 0;
    turbo.currentStateX = false;
    turbo.currentStateY = false;    
}

void updateButtons() {
    unsigned long currentMillis = millis();//连发设置

    // 直接读取按键状态 (INPUT_PULLUP: 按下=0, 松开=1)
    //buttons.A      = !digitalRead(A_BUTTON);
    //buttons.B      = !digitalRead(B_BUTTON);
    //buttons.LEFT   = !digitalRead(LEFT_BUTTON);
    //buttons.RIGHT  = !digitalRead(RIGHT_BUTTON);
    //buttons.UP     = !digitalRead(UP_BUTTON);
    //buttons.DOWN   = !digitalRead(DOWN_BUTTON);
    buttons.START  = !digitalRead(START_BUTTON);
    buttons.SELECT = !digitalRead(SELECT_BUTTON);

    // ========== 读取摇杆 X/Y 信号 ==========     追加代码
    int xValue = analogRead(JOYSTICK_X_PIN);  // 范围 0~4095
    int yValue = analogRead(JOYSTICK_Y_PIN);
    
    // 计算偏离中心的值   追加代码
    int xDiff = xValue - JOYSTICK_CENTER_X;
    int yDiff = yValue - JOYSTICK_CENTER_Y;
         
    // 上方向（原左方向）：X < 中心 - 死区
    buttons.DOWN = (xDiff < -JOYSTICK_DEADZONE) ? 1 : 0;

    // 右方向（原上方向）：Y < 中心 - 死区（摇杆上推是 Y 减小）
    buttons.LEFT = (yDiff < -JOYSTICK_DEADZONE) ? 1 : 0;

    // 下方向（原右方向）：X > 中心 + 死区 
    buttons.UP = (xDiff > JOYSTICK_DEADZONE) ? 1 : 0;

    // 左方向（原下方向）：Y > 中心 + 死区
    buttons.RIGHT = (yDiff > JOYSTICK_DEADZONE) ? 1 : 0;

    
    
        // ========== A键：普通A键 ==========
    bool aPhysical = !digitalRead(A_BUTTON);
    
    // ========== X键：连发A ==========
    bool xPressed = !digitalRead(X_BUTTON);
    bool xTurboState = false;
    if (xPressed) {
        // X键按下，独立处理X的连发
        if (currentMillis - turbo.lastTickX >= TURBO_INTERVAL_MS) {
            turbo.currentStateX = !turbo.currentStateX;
            turbo.lastTickX = currentMillis;
        }
        xTurboState = turbo.currentStateX;
    } else {
        // X键释放，重置状态
        turbo.currentStateX = false;
        turbo.lastTickX = 0;
        xTurboState = false;
    }
    
    // ========== B键：普通B键 ==========
    bool bPhysical = !digitalRead(B_BUTTON);
    
    // ========== Y键：连发B ==========
    bool yPressed = !digitalRead(Y_BUTTON);
    bool yTurboState = false;
    if (yPressed) {
        // Y键按下，独立处理Y的连发
        if (currentMillis - turbo.lastTickY >= TURBO_INTERVAL_MS) {
            turbo.currentStateY = !turbo.currentStateY;
            turbo.lastTickY = currentMillis;
        }
        yTurboState = turbo.currentStateY;
    } else {
        // Y键释放，重置状态
        turbo.currentStateY = false;
        turbo.lastTickY = 0;
        yTurboState = false;
    }
    
    // 组合按键状态
    buttons.A = aPhysical || xTurboState;
    buttons.B = bPhysical || yTurboState;
}

// ================ 清除屏幕边缘，进入游戏前调用 ================
// ================ 清除屏幕边缘，进入游戏前调用 ================
void clearScreenForGame() {
    tft.waitDMA();
    
    tft.startWrite();
    tft.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, TFT_BLACK);
    tft.endWrite();
    
    if (frame_buf[0]) {
        memset(frame_buf[0], 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    }
    if (frame_buf[1]) {
        memset(frame_buf[1], 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    }
    
    uint8_t dummy;
    while (xQueueReceive(frame_queue, &dummy, 0) == pdTRUE) {}
}

bool tryInitSD() {
    // 使用独立 SPI 总线初始化 SD
    sdSPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, sdSPI, SD_FREQ)) {
        Serial.println("SD card init failed or not inserted");
        sdCardAvailable = false;
        return false;
    }
    
    Serial.println("SD card initialized");
    sdCardAvailable = true;
    return true;
}

void initializeSD() {
    tryInitSD();
}

// ================ ROM 文件扫描 ================
void scanROMFiles() {
    romList.clear();
    
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory");
        return;
    }
    
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        
        if (!entry.isDirectory()) {
            String filename = entry.name();
            
            // 跳过 macOS 元数据文件 (以 ._ 开头)
            String basename = filename;
            int lastSlash = filename.lastIndexOf('/');
            if (lastSlash >= 0) {
                basename = filename.substring(lastSlash + 1);
            }
            if (basename.startsWith("._")) {
                entry.close();
                continue;
            }
            
            // 检查是否为 .nes 文件
            if (filename.endsWith(".nes") || filename.endsWith(".NES") || 
                filename.endsWith(".Nes")) {
                // 确保路径以 / 开头
                if (!filename.startsWith("/")) {
                    filename = "/" + filename;
                }
                romList.push_back(filename);
                Serial.printf("Found ROM: %s\n", filename.c_str());
            }
        }
        entry.close();
    }
    root.close();
    
    Serial.printf("Total ROMs found: %d\n", romList.size());
    
    // 排序 ROM 列表
    std::sort(romList.begin(), romList.end());
}

// ================ 主菜单绘制 ================
void drawMainMenu() {
    tft.fillScreen(MENU_BG_COLOR);
    
    // 获取屏幕实际尺寸
    int screen_w = tft.width();   // 480
    int screen_h = tft.height();  // 320
    
    // ===== 头部标题 =====
    tft.fillRect(0, 0, screen_w, 40, MENU_HEADER_COLOR);
    tft.drawRect(0, 0, screen_w, 40, MENU_BORDER_COLOR);
    
    // 绘制标题（居中）
    tft.setTextColor(MENU_TITLE_COLOR);
    tft.setTextSize(2);
    int title_x = (screen_w - 120) / 2;
    tft.setCursor(title_x, 12);
    tft.print("熊师傅-NES");
    
    // 装饰线
    tft.drawFastHLine(40, 38, screen_w - 80, MENU_BORDER_COLOR);
    
    // ===== 游戏列表区域（10行，边框高度调大） =====
    int listStartY = 48;
    int itemHeight = 22;                // 行高22，10行总高220
    int listWidth = screen_w - 80;      // 400
    int listX = 40;
    
    // 绘制列表边框（高度 = 10 × 22 + 8 = 228，边框更大）
    tft.drawRect(listX - 2, listStartY - 2, listWidth + 4, ITEMS_PER_PAGE * itemHeight + 8, MENU_BORDER_COLOR);
    
    if (romList.empty()) {
        tft.setTextColor(MENU_HINT_COLOR);
        tft.setTextSize(1);
        if (!sdCardAvailable) {
            tft.setCursor((screen_w - 140) / 2, listStartY + 40);
            tft.print("No SD card detected");
            tft.setCursor((screen_w - 200) / 2, listStartY + 60);
            tft.print("Please insert SD card with");
            tft.setCursor((screen_w - 150) / 2, listStartY + 75);
            tft.print(".nes ROM files");
            tft.setCursor((screen_w - 120) / 2, listStartY + 100);
            tft.setTextColor(MENU_ARROW_COLOR);
            tft.print("Press A to retry");
        } else {
            tft.setCursor((screen_w - 220) / 2, listStartY + 60);
            tft.print("No ROM files found on SD card");
            tft.setCursor((screen_w - 150) / 2, listStartY + 80);
            tft.print("Please add .nes files");
        }
    } else {
        int totalPages = (romList.size() + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
        int currentPage = scrollOffset / ITEMS_PER_PAGE + 1;
        
        tft.setTextSize(1);
        
        for (int i = 0; i < ITEMS_PER_PAGE; i++) {
            int romIndex = scrollOffset + i;
            if (romIndex >= (int)romList.size()) break;
            
            int itemY = listStartY + i * itemHeight;
            
            // 高亮选中项
            if (romIndex == selectedIndex) {
                tft.fillRect(listX, itemY + 3, listWidth, itemHeight - 4, MENU_HIGHLIGHT_BG);
                
                // 右侧箭头
                tft.setTextColor(MENU_ARROW_COLOR);
                tft.setCursor(listX + listWidth - 25, itemY + 7);
                tft.print("<");
                // 左侧箭头
                tft.setCursor(listX + 8, itemY + 7);
                tft.print(">");
                
                tft.setTextColor(MENU_TITLE_COLOR);
            } else {
                tft.setTextColor(MENU_TEXT_COLOR);
            }
            
            // 游戏名称
            String displayName = romList[romIndex];
            if (displayName.startsWith("/")) {
                displayName = displayName.substring(1);
            }
            int dotPos = displayName.lastIndexOf('.');
            char serialNum[8];
            snprintf(serialNum, sizeof(serialNum), "%03d", romIndex + 1);
            if (dotPos > 0) {
                displayName = String(serialNum) + "  " + displayName.substring(0, dotPos);
            }
            if (displayName.length() > 38) {
                displayName = displayName.substring(0, 35) + "...";
            }
            
            tft.setCursor(listX + 25, itemY + 7);
            tft.print(displayName);
        }
        
        // 分页信息
        tft.setTextColor(MENU_HINT_COLOR);
        int pageInfo_x = screen_w - 70;
        int pageInfo_y = listStartY + ITEMS_PER_PAGE * itemHeight - 4;
        tft.fillRect(pageInfo_x - 5, pageInfo_y - 12, 70, 24, MENU_BG_COLOR);
        tft.setCursor(pageInfo_x, pageInfo_y);
        char pageInfo[16];
        snprintf(pageInfo, sizeof(pageInfo), "%d/%d", currentPage, totalPages);
        tft.print(pageInfo);
    }
    
    // ===== 底部操作提示 =====
    int hintY = screen_h - 32;
    tft.fillRect(0, hintY, screen_w, 32, MENU_HEADER_COLOR);
    tft.drawFastHLine(0, hintY, screen_w, MENU_BORDER_COLOR);
    
    tft.setTextColor(MENU_HINT_COLOR);
    tft.setTextSize(1);
    int hint_x = (screen_w - 260) / 2;
    tft.setCursor(hint_x, hintY + 10);
    tft.print("UP/DOWN: Select    START: Play Game");
}

// ================ 暂停菜单绘制 ================
void drawPauseMenu() {
    // 半透明遮罩
    for (int y = 0; y < DISPLAY_HEIGHT; y += 2) {
        tft.drawFastHLine(0, y, DISPLAY_WIDTH, PAUSE_OVERLAY_COLOR);
    }
    
    // 暂停菜单框
    int menuWidth = 240;
    int menuHeight = 160;
    int menuX = (DISPLAY_WIDTH - menuWidth) / 2;
    int menuY = (DISPLAY_HEIGHT - menuHeight) / 2;
    
    tft.fillRect(menuX, menuY, menuWidth, menuHeight, MENU_BG_COLOR);
    tft.drawRect(menuX, menuY, menuWidth, menuHeight, MENU_BORDER_COLOR);
    tft.drawRect(menuX + 1, menuY + 1, menuWidth - 2, menuHeight - 2, MENU_BORDER_COLOR);
    
    tft.setTextColor(MENU_TITLE_COLOR);
    tft.setTextSize(2);
    tft.setCursor(menuX + (menuWidth - 80) / 2, menuY + 15);
    tft.print("PAUSED");
    
    tft.drawFastHLine(menuX + 20, menuY + 38, menuWidth - 40, MENU_BORDER_COLOR);
    
    const char* options[] = {"Continue", "Save State", "Load State", "Exit to Menu"};
    int optionCount = 4;
    tft.setTextSize(1);
    
    for (int i = 0; i < optionCount; i++) {
        int optY = menuY + 50 + i * 24;
        
        if (i == pauseMenuIndex) {
            tft.fillRect(menuX + 20, optY - 2, menuWidth - 40, 20, MENU_HIGHLIGHT_BG);
            tft.setTextColor(MENU_ARROW_COLOR);
            tft.setCursor(menuX + 30, optY + 5);
            tft.print("> ");
            tft.setTextColor(MENU_TITLE_COLOR);
        } else {
            tft.setTextColor(MENU_TEXT_COLOR);
            tft.setCursor(menuX + 30, optY + 5);
            tft.print("  ");
        }
        tft.print(options[i]);
    }
    
    tft.setTextColor(MENU_HINT_COLOR);
    tft.setCursor(menuX + (menuWidth - 200) / 2, menuY + menuHeight - 18);
    tft.print("UP/DOWN: Select  A: OK");
}

// ================ 菜单输入处理 ================
void handleMenuInput() {
    unsigned long now = millis();
    if (now - lastButtonTime < BUTTON_DEBOUNCE) return;
    
    updateButtons();
    
    if (romList.empty()) {
        // 无 ROM 或无 SD 卡时，A 键重试 SD 初始化
        if (buttons.A) {
            lastButtonTime = now;
            SD.end();
            delay(100);
            if (tryInitSD()) {
                scanROMFiles();
            }
            drawMainMenu();
        }
        return;
    }
    
    bool buttonPressed = false;
    // ========== 变量声明放在函数开头 ==========
    int totalGames = (int)romList.size();
    int itemsPerPage = ITEMS_PER_PAGE;
    int totalPages = (totalGames + itemsPerPage - 1) / itemsPerPage;
    int currentPage = scrollOffset / itemsPerPage + 1;
     // 计算当前页的起始和结束索引
    int pageStart = scrollOffset;
    int pageEnd = scrollOffset + itemsPerPage - 1;
    if (pageEnd >= totalGames) pageEnd = totalGames - 1;
    
    // ========== 上移选择（只在当前页内移动）==========
    if (buttons.UP) {
        if (selectedIndex > pageStart) {
            selectedIndex--;
            buttonPressed = true;
        }
        // 如果已经在页顶，不做任何操作（不跳转到上一页）
    }
    
    // ========== 下移选择（只在当前页内移动）==========
    if (buttons.DOWN) {
        if (selectedIndex < pageEnd) {
            selectedIndex++;
            buttonPressed = true;
        }
        // 如果已经在页底，不做任何操作（不跳转到下一页）
    }
    
    // ========== 右键翻页（下一页） ==========
    if (buttons.RIGHT) {
        if (currentPage < totalPages) {
            // 跳转到下一页的第一个游戏
            selectedIndex = currentPage * itemsPerPage;
            // 确保不超出范围
            if (selectedIndex >= totalGames) {
                selectedIndex = totalGames - 1;
            }
            scrollOffset = selectedIndex - (selectedIndex % itemsPerPage);
            if (scrollOffset < 0) scrollOffset = 0;
            buttonPressed = true;
        }
    }
    
    // ========== 左键翻页（上一页） ==========
    if (buttons.LEFT) {
        if (currentPage > 1) {
            // 跳转到上一页的第一个游戏
            selectedIndex = (currentPage - 2) * itemsPerPage;
            if (selectedIndex < 0) selectedIndex = 0;
            scrollOffset = selectedIndex - (selectedIndex % itemsPerPage);
            if (scrollOffset < 0) scrollOffset = 0;
            buttonPressed = true;
        }
    }

    // 确认选择
    if (buttons.START || buttons.A) {
        if (loadSelectedROM()) {
            currentState = STATE_PLAYING;
        }
        buttonPressed = true;
    }
    
    if (buttonPressed) {
        lastButtonTime = now;
        // 使用局部刷新减少闪烁
        drawMenuList();
    }
}

// ================ 绘制菜单列表区域（局部刷新） ================

// ================ 绘制菜单列表区域（局部刷新） ================
void drawMenuList() {
    // 获取屏幕实际尺寸
    int screen_w = tft.width();
    
    // 与 drawMainMenu 保持一致的参数
    int listStartY = 48;
    int itemHeight = 22;
    int listWidth = screen_w - 80;
    int listX = 40;
    
    // 清除列表区域（包括边框内部）
    tft.fillRect(listX, listStartY, listWidth, ITEMS_PER_PAGE * itemHeight + 8, MENU_BG_COLOR);
    
    if (romList.empty()) return;
    
    tft.setTextSize(1);
    
    for (int i = 0; i < ITEMS_PER_PAGE; i++) {
        int romIndex = scrollOffset + i;
        if (romIndex >= (int)romList.size()) break;
        
        int itemY = listStartY + i * itemHeight;
        
        if (romIndex == selectedIndex) {
            tft.fillRect(listX, itemY + 3, listWidth, itemHeight - 4, MENU_HIGHLIGHT_BG);
            
            tft.setTextColor(MENU_ARROW_COLOR);
            tft.setCursor(listX + listWidth - 25, itemY + 7);
            tft.print("<");
            tft.setCursor(listX + 8, itemY + 7);
            tft.print(">");
            
            tft.setTextColor(MENU_TITLE_COLOR);
        } else {
            tft.setTextColor(MENU_TEXT_COLOR);
        }
        
        String displayName = romList[romIndex];
        if (displayName.startsWith("/")) {
            displayName = displayName.substring(1);
        }
        int dotPos = displayName.lastIndexOf('.');
        char serialNum[8];
        snprintf(serialNum, sizeof(serialNum), "%03d", romIndex + 1);
        if (dotPos > 0) {
            displayName = String(serialNum) + "  " + displayName.substring(0, dotPos);
        }
        if (displayName.length() > 38) {
            displayName = displayName.substring(0, 35) + "...";
        }
        
        tft.setCursor(listX + 25, itemY + 7);
        tft.print(displayName);
    }
    
    // 重绘边框（高度与 drawMainMenu 一致）
    tft.drawRect(listX - 2, listStartY - 2, listWidth + 4, ITEMS_PER_PAGE * itemHeight + 8, MENU_BORDER_COLOR);
    
    // 更新分页信息
    int totalPages = (romList.size() + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    int currentPage = scrollOffset / ITEMS_PER_PAGE + 1;
    tft.setTextColor(MENU_HINT_COLOR);
    int pageInfo_x = screen_w - 70;
    int pageInfo_y = listStartY + ITEMS_PER_PAGE * itemHeight - 4;
    tft.fillRect(pageInfo_x - 5, pageInfo_y - 12, 70, 24, MENU_BG_COLOR);
    tft.setCursor(pageInfo_x, pageInfo_y);
    char pageInfo[16];
    snprintf(pageInfo, sizeof(pageInfo), "%d/%d", currentPage, totalPages);
    tft.print(pageInfo);
}


// ================ 暂停输入处理 ================
void handlePauseInput() {
    unsigned long now = millis();
    if (now - lastButtonTime < BUTTON_DEBOUNCE) return;
    
    updateButtons();
    
    bool buttonPressed = false;
    
    if (buttons.UP) {
        if (pauseMenuIndex > 0) {
            pauseMenuIndex--;
            buttonPressed = true;
        }
    }
    
    if (buttons.DOWN) {
        if (pauseMenuIndex < 3) {  // 现在有4个选项 (0-3)
            pauseMenuIndex++;
            buttonPressed = true;
        }
    }
    
    if (buttons.A || buttons.START) {
        // 等待按键释放
        delay(100);
        while (digitalRead(A_BUTTON) == LOW || digitalRead(START_BUTTON) == LOW) {
            delay(10);
        }
        delay(50);
        
        if (pauseMenuIndex == 0) {
            // Continue - 清屏后继续游戏
            clearScreenForGame();
            resetFrameScheduler(3);
            gameRunning = true;  // 恢复音频
            currentState = STATE_PLAYING;
        } else if (pauseMenuIndex == 1) {
            // Save State
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(MENU_TITLE_COLOR);
            tft.setTextSize(2);
            tft.setCursor(80, 110);
            tft.print("Saving...");
            
            char savePath[128];
            getSaveStatePath(savePath, sizeof(savePath));
            if (nes.saveState(savePath)) {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(0x07E0);  // 绿色成功提示
                tft.setTextSize(2);
                tft.setCursor(60, 110);
                tft.print("State Saved!");
                delay(1000);
            } else {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(0xF800);  // 红色错误提示
                tft.setTextSize(2);
                tft.setCursor(60, 110);
                tft.print("Save Failed!");
                delay(1500);
            }
            
            // 返回游戏
            clearScreenForGame();
            resetFrameScheduler(3);
            gameRunning = true;
            currentState = STATE_PLAYING;
        } else if (pauseMenuIndex == 2) {
            // Load State
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(MENU_TITLE_COLOR);
            tft.setTextSize(2);
            tft.setCursor(80, 110);
            tft.print("Loading...");
            
            char savePath[128];
            getSaveStatePath(savePath, sizeof(savePath));
            if (nes.loadState(savePath)) {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(0x07E0);  // 绿色成功提示
                tft.setTextSize(2);
                tft.setCursor(60, 110);
                tft.print("State Loaded!");
                delay(1000);
            } else {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(0xF800);  // 红色错误提示
                tft.setTextSize(2);
                tft.setCursor(60, 110);
                tft.print("Load Failed!");
                tft.setTextColor(MENU_HINT_COLOR);
                tft.setTextSize(1);
                tft.setCursor(50, 140);
                tft.print("No save state found");
                delay(1500);
            }
            
            // 返回游戏
            clearScreenForGame();
            resetFrameScheduler(3);
            gameRunning = true;
            currentState = STATE_PLAYING;
        } else {
            // Exit to Menu
            returnToMainMenu();
        }
        return;  // 直接返回，不需要后续处理
    }
    
    // B 按钮也返回游戏
    if (buttons.B) {
        delay(100);
        while (digitalRead(B_BUTTON) == LOW) {
            delay(10);
        }
        delay(50);
        clearScreenForGame();
        resetFrameScheduler(3);
        gameRunning = true;  // 恢复音频
        currentState = STATE_PLAYING;
        return;
    }
    
    if (buttonPressed) {
        lastButtonTime = now;
        if (currentState == STATE_PAUSED) {
            drawPauseMenu();
        }
    }
}

// ================ 加载选中的ROM ================
bool loadSelectedROM() {
    if (selectedIndex < 0 || selectedIndex >= (int)romList.size()) {
        return false;
    }
    
    const char* romPath = romList[selectedIndex].c_str();
    Serial.printf("Loading ROM: %s\n", romPath);
    
    // 显示加载中
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(MENU_TITLE_COLOR);
    tft.setTextSize(2);
    tft.setCursor(100, 110);
    tft.print("Loading...");
    
    if (!nes.loadROM(romPath)) {
        Serial.printf("Failed to load ROM: %s\n", romPath);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(0xF800);  // 红色错误提示
        tft.setTextSize(2);
        tft.setCursor(60, 100);
        tft.print("Load Failed!");
        tft.setTextColor(MENU_HINT_COLOR);
        tft.setTextSize(1);
        tft.setCursor(40, 130);
        tft.print("Unsupported mapper or bad ROM");
        tft.setCursor(50, 150);
        tft.print("Supported: Mapper 0-4");
        tft.setCursor(60, 170);
        tft.print("Returning to menu...");
        delay(3000);
        tft.fillScreen(MENU_BG_COLOR);
        drawMainMenu();
        return false;
    }
    
    nes.reset();
    
    // 应用抽帧开关设置
    nes.setFrameskipEnabled(ENABLE_FRAMESKIP);
    
    // 设置 PPU 的帧缓冲
    nes.getPPU().frameBuffer = frame_buf[render_buf_idx];
    
    // 彻底清屏为黑色 (清除菜单残留)
    clearScreenForGame();
    
    // 开始运行游戏并启用音频
    resetFrameScheduler(3);
    game_start_ms = millis();
    last_rendered_ms = 0;
    gameRunning = true;
    Serial.println("ROM loaded successfully");

    gameJustEntered = true;
    return true;
}

// ================ 返回主菜单 ================
void returnToMainMenu() {
    // 停止游戏并静音
    gameRunning = false;
    muteAudio();
    
    currentState = STATE_MENU;
    selectedIndex = 0;
    scrollOffset = 0;
    pauseMenuIndex = 0;
    
    // 清屏并重绘菜单
    tft.fillScreen(MENU_BG_COLOR);
    drawMainMenu();
}

void loadROM() {
    // 现在使用菜单选择，这里只是扫描ROM列表
    if (sdCardAvailable) {
        scanROMFiles();
    }
}

// ---------------- Audio (I2S) ----------------
static void initializeAudio() {
    // Install and configure I2S driver
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true
    };

    esp_err_t res = i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0, NULL);
    if (res != ESP_OK) {
        Serial.printf("I2S driver install failed: %d\n", res);
    } else {
        Serial.println("I2S driver installed");
    }

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCLK_PIN,
        .data_out_num = I2S_DATA_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin((i2s_port_t)I2S_NUM, &pin_config);
    i2s_zero_dma_buffer((i2s_port_t)I2S_NUM);

    // Let APU know sample rate (if needed)
    nes.apu.setSampleRate(AUDIO_SAMPLE_RATE);

    // APU 独占 Core 0，仿真+渲染+推屏全在 Core 1 (loop)
    // APU::clock() 内部会在缓冲区满时阻塞写入 I2S
    // I2S DMA 的消费速度 (44100 Hz) 自然控制 APU 时钟速度
    xTaskCreatePinnedToCore(apu_task, "APU", 2048, &nes.apu, 1, NULL, 0);
}

// APU task: 像 Anemoia 一样无限循环调用 clock()
// 每 2 次循环调用一次 clock()，匹配 NES APU 时序（APU 时钟 = CPU 时钟 / 2）
// I2S 写入阻塞会自然控制速度
static void apu_task(void* arg) {
    APU* apu = (APU*)arg;
    while (1) {
        if (gameRunning) {
            apu->clock();
        } else {
            // 游戏未运行时，输出静音并稍作等待
            vTaskDelay(1);
        }
    }
}

// 静音音频输出
static void muteAudio() {
    // 清空 I2S DMA 缓冲区
    i2s_zero_dma_buffer((i2s_port_t)I2S_NUM);
    // 增加延迟确保缓冲区完全清空
    delay(50);
    i2s_zero_dma_buffer((i2s_port_t)I2S_NUM);
}

// ================ 主程序 ================
void setup() {
    initializeSerial();
    initializeScreen();
    initializeButtons();
    initializeSD();
    loadROM();  // 扫描 ROM 文件列表
        // 初始化摇杆 ADC    追加代码
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 初始化摇杆引脚    追加代码
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);

        // 临时调试：读取摇杆中心值   追加代码
    Serial.begin(921600);
    delay(200);  // 给时间打开串口监视器
    int x = analogRead(JOYSTICK_X_PIN);
    int y = analogRead(JOYSTICK_Y_PIN);
    Serial.printf("X=%d, Y=%d\n", x, y);
    JOYSTICK_CENTER_X=x;
    JOYSTICK_CENTER_Y=y;


    // 初始化音频 (I2S) 并在另一个 CPU core 上运行音频任务
    initializeAudio();
    tft.setFont(&fonts::efontCN_16_b);  // 设置简体中文字库  追加代码

    // 创建显示任务在 Core 0
    frame_queue = xQueueCreate(1, sizeof(uint8_t));
    if (frame_queue) {
        xTaskCreatePinnedToCore(display_task, "Display", 4096, nullptr, 1, nullptr, 0);
    }
    
    // 显示主菜单
    currentState = STATE_MENU;
    drawMainMenu();
}

void loop() {
    // 根据当前状态处理不同逻辑
    switch (currentState) {
        case STATE_MENU:
            handleMenuInput();
            delay(50);  // 菜单模式降低刷新率，节省资源
            return;
            
        case STATE_PAUSED:
            handlePauseInput();
            delay(50);
            return;
            
        case STATE_PLAYING:
            // 正常游戏逻辑
            if (gameJustEntered) {
                clearScreenForGame();   // ⭐ 强制清左右黑边
                gameJustEntered = false;
            }
            break;
    }

    // ===== Anemoia 风格游戏运行逻辑 =====
    // 帧级别调度：目标 60Hz 仿真 (16639µs/帧)
    #define FRAME_TIME_US 16639
    static bool pauseKeyReleased = true;  // 暂停组合键是否已释放

    // 更新按键输入
    updateButtons();
    
    // 检测 START + SELECT 组合键进入暂停菜单
    if (buttons.START && buttons.SELECT) {
        if (pauseKeyReleased) {
            pauseKeyReleased = false;
            currentState = STATE_PAUSED;
            pauseMenuIndex = 0;
            // 暂停游戏并静音
            gameRunning = false;
            muteAudio();
            // 等待按键释放，防止立即退出暂停
            delay(100);
            while (digitalRead(START_BUTTON) == LOW || digitalRead(SELECT_BUTTON) == LOW) {
                delay(10);
            }
            delay(100);
            drawPauseMenu();
            return;
        }
    } else {
        pauseKeyReleased = true;
    }
    
    uint8_t controllerState = 0;
    if (buttons.A)      controllerState |= 0x01;
    if (buttons.B)      controllerState |= 0x02;
    if (buttons.SELECT) controllerState |= 0x04;
    if (buttons.START)  controllerState |= 0x08;
    if (buttons.UP)     controllerState |= 0x10;
    if (buttons.DOWN)   controllerState |= 0x20;
    if (buttons.LEFT)   controllerState |= 0x40;
    if (buttons.RIGHT)  controllerState |= 0x80;
    nes.setController(0, controllerState);

    // 初始化帧计时
    if (next_frame_us == 0) next_frame_us = esp_timer_get_time();

    // 自适应抽帧：只有在主循环已经落后于目标帧节奏时才跳过本帧渲染。
    int64_t frameLagUs = (int64_t)esp_timer_get_time() - (int64_t)next_frame_us;
    bool frameskipPhaseAllowsSkip = (frameskip_phase == 0 ||
                                     frameskip_phase == 2 ||
                                     frameskip_phase == 4 ||
                                     frameskip_phase == 6 ||
                                     frameskip_phase == 8);
    bool shouldSkipFrame = ENABLE_FRAMESKIP &&
                           (force_render_frames == 0) &&
                           (consecutive_skipped_frames == 0) &&
                           frameskipPhaseAllowsSkip &&
                           (frameLagUs > (FRAME_TIME_US / 2));
    nes.requestFrameSkip(shouldSkipFrame);

    // 执行一帧
    uint32_t emu0 = micros();
    //runFrame(); // 方法1: 行级调度
    nes.clock(); // 方法2: 帧级调度
    last_emulation_us = micros() - emu0;
    
    // 入队帧缓冲用于 DMA 显示
    tryEnqueueFrame();

    if (last_rendered_ms == 0 && millis() - game_start_ms > 3500) {
        gameRunning = false;
        muteAudio();
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(0xF800);
        tft.setTextSize(2);
        tft.setCursor(58, 95);
        tft.print("Game Start Failed");
        tft.setTextColor(MENU_HINT_COLOR);
        tft.setTextSize(1);
        tft.setCursor(42, 130);
        tft.print("Unsupported or unstable ROM");
        tft.setCursor(65, 150);
        tft.print("Returning to menu...");
        delay(3000);
        tft.fillScreen(MENU_BG_COLOR);
        drawMainMenu();
        currentState = STATE_MENU;
        return;
    }

    if (shouldSkipFrame) {
        consecutive_skipped_frames++;
    } else {
        consecutive_skipped_frames = 0;
        if (force_render_frames > 0) force_render_frames--;
    }
    frameskip_phase++;
    if (frameskip_phase >= 9) frameskip_phase = 0;

    // FPS 统计
    fps_count++;
    uint32_t curMs = millis();
    if (fps_last_ms == 0) fps_last_ms = curMs;
    if (curMs - fps_last_ms >= 1000) {
        FPS_PRINT("FPS:%u  EMU:%uus  DMA:%uus\n",
            fps_count, last_emulation_us, last_dma_us);
        fps_count = 0;
        fps_last_ms = curMs;
    }

    // 帧限制
    uint64_t now = esp_timer_get_time();
    if (now < next_frame_us) {
        ets_delay_us(next_frame_us - now);
    }
    next_frame_us += FRAME_TIME_US;
    #undef FRAME_TIME_US
}
