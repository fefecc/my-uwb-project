#ifndef APP_APP_APP_H_
#define APP_APP_APP_H_

#include <stdbool.h>

#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

AppMode App_DetectBootMode(void);
bool App_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_APP_APP_H_ */
