/**
 * Based on Espressif examples: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/uart/uart_echo/main/uart_echo_example_main.c
 * And ported library from https://github.com/mandulaj/PZEM-004T-v30
 *
 */
#include "pzem003.h"

/* Declare static func in .c file (linker warnings) */
static uint16_t crc16(const uint8_t *data, uint16_t len);

uint16_t _lastRead = 0; /* Last time values were updated */

/**
 * @brief Initialize the UART, configured via struct pzemSetup_t
 * @param pzSetup
 */
void PzemInit( pzem_setup_t *pzSetup )
{
    static const char *LOG_TAG = "PZ_INIT";

    ESP_LOGI( LOG_TAG, "Initializing UART" );

    const uart_port_t _uart_num = pzSetup->pzem_uart;
    const int uart_buffer_size = ( 1024 * 2 );

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = PZ_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_RTC,
    };

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
#if CONFIG_UART_MORE_PRIO
    intr_alloc_flags = ESP_INTR_FLAG_LEVEL3;
#endif

    ESP_LOGI( LOG_TAG, "UART set pins, mode and install driver." );

    /* Install UART driver using an event queue here */
    ESP_ERROR_CHECK( uart_driver_install( _uart_num, uart_buffer_size, 0, 0, NULL, intr_alloc_flags ) );

    /* Configure UART parameters */
    ESP_ERROR_CHECK( uart_param_config( _uart_num, &uart_config ) );

    /* Set UART pins(TX: , RX: , RTS: -1, CTS: -1) */
    ESP_ERROR_CHECK( uart_set_pin( _uart_num, pzSetup->pzem_tx_pin, pzSetup->pzem_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );

}


/**
 * @brief Read response
 * @param pzSetup
 * @param resp
 * @param len
 * @return uint16_t
 */
uint16_t PzemReceive( pzem_setup_t *pzSetup, uint8_t *resp, uint16_t len )
{
    static const char *LOG_TAG = "PZ_RECEIVE";

    /* Configure a temporary buffer for the incoming data */
    uint16_t rxBytes = uart_read_bytes( pzSetup->pzem_uart, resp, len, pdMS_TO_TICKS( PZ_READ_TIMEOUT ) );

    if ( rxBytes > 0 ) {
        resp[ rxBytes ] = 0;
        ESP_LOGV( LOG_TAG, "Read %d  bytes: '%s'", rxBytes, resp );
        ESP_LOG_BUFFER_HEXDUMP( LOG_TAG, resp, rxBytes, ESP_LOG_VERBOSE );
    }

    return rxBytes;
}


/**
 * @brief In case you forgot the address
 * @param update
 * @return uint8_t
 */

/**
 * @brief Change the default address of the pzem module
 * @return  true if succeeded
 */

/**
 * @brief Energy counter does not reset af restart, this function can reset the counter
 * @param pzSetup
 * @return bool
 */


/**
 * @brief Send 8Bit command
 * @param pzSetup
 * @param cmd
 * @param regAddr
 * @param regVal
 * @param check
 * @param slave_addr
 * @return bool
 */
bool PzemSendCmd8( pzem_setup_t *pzSetup, uint8_t cmd, uint16_t regAddr, uint16_t regVal, bool check, uint16_t slave_addr )
{
    static const char *LOG_TAG = "PZ_SEND8";

    /* send and receive buffers memory allocation */
    uint8_t txdata[TX_BUF_SIZE] = {0};
    uint8_t rxdata[RX_BUF_SIZE] = {0};

    memset(txdata, 0, sizeof(txdata));
    memset(rxdata, 0, sizeof(rxdata));

    if ( ( slave_addr == 0xFFFF ) ||
            ( slave_addr < 0x01 ) ||
            ( slave_addr > 0xF7 ) ) {
        slave_addr = pzSetup->pzem_addr;
    }

    txdata[ 0 ] = slave_addr;
    txdata[ 1 ] = cmd;
    txdata[ 2 ] = ( regAddr >> 8 ) & 0xFF;
    txdata[ 3 ] = ( regAddr ) & 0xFF;
    txdata[ 4 ] = ( regVal >> 8 ) & 0xFF;
    txdata[ 5 ] = ( regVal ) & 0xFF;

    /* Add CRC to array */
    (void)PzemSetCRC( txdata, TX_BUF_SIZE );

    const int txBytes = uart_write_bytes( pzSetup->pzem_uart, txdata, TX_BUF_SIZE );

    ESP_LOGV( LOG_TAG, "Wrote %d bytes", txBytes );
    ESP_LOG_BUFFER_HEXDUMP( LOG_TAG, txdata, txBytes, ESP_LOG_VERBOSE );

    if ( check ) {
        if ( !PzemReceive( pzSetup, rxdata, RX_BUF_SIZE ) ) { /* if check enabled, read the response */
            return false;
        }

        /* Check if response is same as send */
        for (uint8_t i = 0; i < 8; i++) {
            if ( txdata[ i ] != rxdata[ i ] ) {
                return false;
            }
        }
    }

    return true;
}


/**
 * @brief Retreive all measurements, leave 200ms between intervals
 * @param pzSetup
 * @param currentValues
 * @return bool
 */
bool PzemGetValues( pzem_setup_t *pzSetup, _current_values_t *pmonValues )
{
    static const char *LOG_TAG = "PZ_GETVALUES";
    if ( ( unsigned long ) ( millis() - _lastRead ) > UPDATE_TIME ) {
        _lastRead = millis();
    } else {
        return true;
    }

    /* Zero all values */
    (void)PzemZeroValues( ( _current_values_t * ) pmonValues );

    uint8_t respbuff[RESP_BUF_SIZE] = {0};
    memset(respbuff, 0, sizeof(respbuff));


    /* Tell the sensor to Read 10 Registers from 0x00 to 0x0A (all values) */
    //if (PzemSendCmd8( pzSetup, CMD_RIR, 0x00, 0x0A, false, 0xFFFF ) == false) {
    if (PzemSendCmd8( pzSetup, CMD_RIR, 0x00, 0x08, false, 0xFFFF ) == false) {
   
        ESP_LOGE(LOG_TAG, "Error writing to registers !!");
    }

    /* Read response from the sensor, if everything goes well we retreived 25 Bytes */
    if ( PzemReceive( pzSetup, respbuff, RESP_BUF_SIZE ) != RESP_BUF_SIZE ) { /* Something went wrong */
        ESP_LOGE(LOG_TAG, "Error RESP_BUF_SIZE !!");
        return false;
    }



    pmonValues->voltage = ( ( uint32_t ) respbuff[ 3 ] << 8 | /* Raw voltage in 0.1V */
                            ( uint32_t ) respbuff[ 4 ] ) / 100.0;
    pmonValues->current = ((uint32_t) respbuff[5] << 8 | // Raw voltage in 0.01V
                            (uint32_t)respbuff[6])/100.0;


    pmonValues->power =   ((uint32_t)respbuff[7] << 8 | // Raw power in 0.1W
                              (uint32_t)respbuff[8] |
                              (uint32_t)respbuff[9] << 24 |
                              (uint32_t)respbuff[10] << 16) / 10.0;

    pmonValues->energy =  ((uint32_t)respbuff[11] << 8 | // Raw Energy in 1Wh
                              (uint32_t)respbuff[12] |
                              (uint32_t)respbuff[13] << 24 |
                              (uint32_t)respbuff[14] << 16) / 1000.0;



    return true;
}

/**
 * @brief Add CRC to 8Bit command
 * @param buf
 * @param len
 */
void PzemSetCRC( uint8_t *buf, uint16_t len )
{
    static const char *TAG = "[SETCRC]";
    uint64_t start = esp_timer_get_time();

    if ( len <= 2 ) { /* sanity check */
        return;
    }

    uint16_t crc = crc16( buf, len - 2 ); /* CRC of data */

    /* Write high and low byte to last two positions */
    buf[ len - 2 ] = crc & 0xFF;          /* Low byte first */
    buf[ len - 1 ] = ( crc >> 8 ) & 0xFF; /* High byte second */

    uint64_t stop = esp_timer_get_time();
    ESP_LOGV(TAG, "Routine crc16() took %llu microseconds", (stop - start));
}

/**
 * @brief Validate CRC
 * @param buf
 * @param len
 * @return
 */

/**
 * @brief Reset all measured values in struct
 * @param currentValues
 */
void PzemZeroValues( _current_values_t *currentValues )
{
    currentValues->alarms = 0;
    currentValues->current = 0.0f;
    currentValues->energy = 0.0f;
    currentValues->frequency = 0.0f;
    currentValues->pf = 0.0f;
    currentValues->power = 0.0f;
    currentValues->voltage = 0.0f;
    currentValues->apparent_power = 0.0f;
    currentValues->reactive_power = 0.0f;
    currentValues->fi = 0.0f;
}

/**
 * @brief Calculate CRC and compare to lookup table
 * @param data
 * @param len
 * @return crc result
 */
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--) {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }

    return crc;
}