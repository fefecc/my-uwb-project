/**
 * @file uwb_slots.c
 * @brief UWB 共享内存池实现
 */

#include "uwb_slots.h"
#include <string.h>

static uwb_slot_t g_slot_pool[UWB_SLOT_NUM];

void UwbSlots_Init(void)
{
    memset(g_slot_pool, 0, sizeof(g_slot_pool));
    for (int i = 0; i < UWB_SLOT_NUM; i++) {
        g_slot_pool[i].owner = UWB_SLOT_FREE;
    }
}

int8_t UwbSlots_Alloc(uwb_slot_owner_t owner)
{
    for (int i = 0; i < UWB_SLOT_NUM; i++) {
        if (g_slot_pool[i].owner == UWB_SLOT_FREE) {
            memset(&g_slot_pool[i], 0, sizeof(uwb_slot_t));
            g_slot_pool[i].owner = owner;
            return (int8_t)i;
        }
    }
    return -1;
}

void UwbSlots_Free(int8_t idx)
{
    if (idx >= 0 && idx < UWB_SLOT_NUM) {
        g_slot_pool[idx].owner = UWB_SLOT_FREE;
    }
}

uwb_slot_t *UwbSlots_Get(int8_t idx)
{
    if (idx >= 0 && idx < UWB_SLOT_NUM) {
        return &g_slot_pool[idx];
    }
    return NULL;
}
