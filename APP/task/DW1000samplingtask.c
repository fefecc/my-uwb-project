#include "DW1000samplingtask.h"
#include "stdio.h"
#include "bphero_uwb.h"
#include "deca_device_api.h"
#include "trilateration.h"
#include "deca_regs.h"
#include "dwm1000_timestamp.h"

// static void Handle_TimeStamp(void);
struct time_timestamp tx_node[MAX_TARGET_NODE];

extern int rx_main(void);
extern int tx_main(void);

void DW1000samplingtask(void *argument)
{
    BPhero_UWB_Message_Init();
    BPhero_UWB_Init(); // dwm1000 init related

    rx_main();
    // tx_main();
}
