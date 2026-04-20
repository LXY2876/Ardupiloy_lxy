/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

void setup();
void loop();
uint8_t  angle=0;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that hal.serial(n) can be initialized
    
    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(5), "SERIAL4");  // 2nd GPS
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
}

void loop(void)
{
    uint8_t test_data[] = {0xff, 0xff,0x01,0x07,0x03,0x2A,0x00,0x00,0x00,0x00,0xC0};
    test_uart(hal.serial(0), "SERIAL0");
    // test_uart(hal.serial(1), "SERIAL1");
    // test_uart(hal.serial(2), "SERIAL2");
    // test_uart(hal.serial(3), "SERIAL3");
    // test_uart(hal.serial(5), "SERIAL4");
    //以下为测试控制总线舵机角度的代码
    angle=(angle+1)%180;
    uint16_t angle_byte=4096*(angle+180)/360;
    test_data[6]=(angle_byte >> 8) & 0xFF; 
    test_data[7]=angle_byte & 0xFF;
    test_data[10]=~(53+test_data[6]+test_data[7])& 0xFF;
    hal.serial(5)->write(test_data, sizeof(test_data));
    
    //以下为测试读取总线舵机当前角度的代码
    uint8_t test_data[] = {0xff, 0xff,0x01,0x04,0x02, 0x38, 0x02,0xBE};
    uint8_t return_data[8];
    hal.serial(5)->write(test_data, sizeof(test_data));
    //读取返回值
    hal.serial(5)->read(return_data, sizeof(test_data));
    if (test_data[0]==0xff && test_data[1]==0xfe){
        int16_t angle=test_data[5]*256+test_data[6];
    }
    //以上为测试某型号总线舵机的代码
    // also do a raw printf() on some platforms, which prints to the
    // debug console
    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));

    hal.scheduler->delay(1000);



}

AP_HAL_MAIN();
