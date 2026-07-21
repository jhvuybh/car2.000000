#include "k230.h"
#include "ti_msp_dl_config.h"

uint8_t k230_get_room(void)
{
    uint8_t receive_data;

    while (DL_UART_isRXFIFOEmpty(K230_INST) == false)
    {
        receive_data = DL_UART_receiveData(K230_INST);

        if (receive_data >= '1' &&
            receive_data <= '8')
        {
            return receive_data - '0';
        }
    }

    return 0;
}        