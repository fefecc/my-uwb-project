#ifndef APP_SERVICE_STORAGE_SERVICE_H_
#define APP_SERVICE_STORAGE_SERVICE_H_

#include <stdbool.h>
#include <stddef.h>

#include "ff.h"
#include "../common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool StorageService_Mount(void);
bool StorageService_OpenNextLog(FIL *file);
bool StorageService_WriteBlock(FIL *file, const uint8_t *data, size_t len);
bool StorageService_WriteNodeAscii(FIL *file, const AppDataNode *node);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVICE_STORAGE_SERVICE_H_ */
