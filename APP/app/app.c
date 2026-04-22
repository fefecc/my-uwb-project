#include "app.h"

#include "../bsp/bsp_key.h"
#include "../bsp/bsp_led.h"
#include "../task/app_tasks.h"

AppMode App_DetectBootMode(void)
{
    BspLed_Init();
    return BspKey_IsPressed() ? APP_MODE_CONFIG : APP_MODE_RUN;
}

bool App_Start(void)
{
    AppMode mode = App_DetectBootMode(); // 当前的模式
    return AppTasks_CreateAll(mode);
}
