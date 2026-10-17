#include <LovyanGFX.hpp>

// 使用LovyanGFX对ESP32进行配置的例子

/// 为你自己的配置创建一个类，派生自LGFX_Device。
class LGFX : public lgfx::LGFX_Device
{
/*
 该类的名称可以从 "LGFX "改为其他名称。
 当与AUTODETECT一起使用时，"LGFX "已经被使用了，所以要把名字改为LGFX以外的名字。
 如果同时使用多个面板，给每个面板一个不同的名字。
 如果改变了类的名称，构造函数的名称也必须改成相同的名称。

 命名方案可以自由决定，但万一配置的数量增加，请将构造函数的名称改为与面板的名称相同。
 例如，如果你在ESP32 DevKit-C中设置了一个ILI9341 SPI连接，你可以使用
  LGFX_DevKitC_SPI_ILI9341
 并将文件名与类名相匹配，这样在使用时就不容易混淆了。
//*/

// 为要连接的屏幕类型准备一个实例。
//lgfx::Panel_GC9A01      _panel_instance;
//lgfx::Panel_GDEW0154M09 _panel_instance;
//lgfx::Panel_HX8357B     _panel_instance;
//lgfx::Panel_HX8357D     _panel_instance;
//lgfx::Panel_ILI9163     _panel_instance;
//lgfx::Panel_ILI9341     _panel_instance;
//lgfx::Panel_ILI9342     _panel_instance;
//lgfx::Panel_ILI9481     _panel_instance;
//lgfx::Panel_ILI9486     _panel_instance;
//lgfx::Panel_ILI9488     _panel_instance;
//lgfx::Panel_IT8951      _panel_instance;
//lgfx::Panel_RA8875      _panel_instance;
//lgfx::Panel_SH110x      _panel_instance; // SH1106, SH1107
//lgfx::Panel_SSD1306     _panel_instance;
//lgfx::Panel_SSD1327     _panel_instance;
//lgfx::Panel_SSD1331     _panel_instance;
//lgfx::Panel_SSD1351     _panel_instance; // SSD1351, SSD1357
//lgfx::Panel_SSD1963     _panel_instance;
//lgfx::Panel_ST7735      _panel_instance;
//lgfx::Panel_ST7735S     _panel_instance;
//lgfx::Panel_ST7789      _panel_instance;
lgfx::Panel_ST7796      _panel_instance;

// 为屏幕所连接的总线类型准备一个实例。
//lgfx::Bus_SPI        _bus_instance;   // SPI
//lgfx::Bus_I2C        _bus_instance;   // I2C
lgfx::Bus_Parallel8  _bus_instance;   // 8 Parallel

// 如果进行背光控制，请提供一个实例。 (如果不需要则删除）。
// lgfx::Light_PWM     _light_instance;

// 为触摸屏类型准备一个实例。 (如果不需要则删除）。
// lgfx::Touch_FT5x06           _touch_instance; // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436
//lgfx::Touch_GSL1680E_800x480 _touch_instance; // GSL_1680E, 1688E, 2681B, 2682B
//lgfx::Touch_GSL1680F_800x480 _touch_instance;
//lgfx::Touch_GSL1680F_480x272 _touch_instance;
//lgfx::Touch_GSLx680_320x320  _touch_instance;
//lgfx::Touch_GT911            _touch_instance;
//lgfx::Touch_STMPE610         _touch_instance;
//lgfx::Touch_TT21xxx          _touch_instance; // TT21100
//lgfx::Touch_XPT2046          _touch_instance;

public:
  LGFX(void)
  {
    // 3. 配置并行总线 (核心改动部分)
    {
      auto cfg = _bus_instance.config();

      // ESP32-S3 专用：使用 LCD_CAM 外设，端口固定为 0
      cfg.port = 0;               // ESP32-S3 的 LCD_CAM 端口
        
      cfg.freq_write = 40000000;  // 并口写入频率，最高 40MHz (16位模式) 或 80MHz (8位模式)
      cfg.freq_read  = 6000000;   // 读取频率
      // 控制引脚 (根据你并口转接板的实际连接修改)
      cfg.pin_wr   = 2;          // 2写入信号 (WR)
      cfg.pin_rd   = -1;          // 读取信号 (RD)，不需要可设为 -1
      cfg.pin_rs   = 1;          //1 命令/数据选择 (RS 或 D/C)

      // 8位数据引脚 D0-D7 (核心：根据你转接板的实际接线修改)
      cfg.pin_d0   = 41;          //41 数据位 0 (LSB)
      cfg.pin_d1   = 40;           //40 数据位 1
      cfg.pin_d2   = 39;           //39 数据位 2
      cfg.pin_d3   = 45;           //45 数据位 3
      cfg.pin_d4   = 47;           //47 数据位 4
      cfg.pin_d5   = 21;           //21 数据位 5
      cfg.pin_d6   = 20;           //20 数据位 6
      cfg.pin_d7   = 19;           //19 数据位 7 (MSB)

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // 配置显示面板控制设置。
      auto cfg = _panel_instance.config();    // 获取屏幕配置的结构。。

      cfg.pin_cs           =    -1; //10 // 连接CS的引脚编号（-1 = 禁用）。
      cfg.pin_rst          =    42; //42 // 连接RST的引脚编号 (-1 = 禁用)
      cfg.pin_busy         =    -1;  // 连接BUSY的引脚编号 (-1 = 禁用)

        // * 下面的设置对每个面板都有一般的默认值，如果你对某个项目不确定，可以把它注释出来并试一试。

      cfg.panel_width      =   320;  // 实际可显示的宽度
      cfg.panel_height     =   480;  // 实际可显示的高度
      cfg.offset_x         =     0;  // 在屏幕的X方向上的偏移量
      cfg.offset_y         =     0;  // 在屏幕的Y方向上的偏移量
      cfg.offset_rotation  =     1;  // 旋转方向的偏移量为0~7（4~7为倒置）。
      cfg.dummy_read_pixel =     8;  // 读取像素前的假读位数量
      cfg.dummy_read_bits  =     1;  // 读取非像素数据前的虚拟读取位数
      cfg.readable         =  true;  // 如果可以读取数据，则设置为true。
      cfg.invert           = true;   // 设定 是否反色，有些屏幕需要设置这个值才能获取正确的颜色
      cfg.rgb_order        = false;  // true 为 RGB false 为 BGR
      cfg.dlen_16bit       = false;  // 如果面板在16位并行或SPI中以16位单位传输数据长度，则设置为true。
      cfg.bus_shared       = false;  // SDカー如果与SD卡共享总线，则设置为true（总线控制由drawJpgFile等执行）。


      _panel_instance.config(cfg);
    }
    setColorDepth(16);    // 强制 16 位 RGB565
    setPanel(&_panel_instance); // 设置要使用的面板。

  }
  };