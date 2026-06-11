#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return len;
    }

    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start) > 50)
        {
            break;
        }
    }

    return len;
}
