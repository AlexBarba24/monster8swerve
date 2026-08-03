/* --------------------------------------------------------------------------
 * TMC5160 software SPI
 *
 * TMC5160 uses SPI mode 3:
 *   CPOL = 1: clock idles high
 *   CPHA = 1: data changes on falling edge and is sampled on rising edge
 *
 * Pins used by the existing project:
 *   SCLK = PE12
 *   MISO = PE13
 *   MOSI = PE14
 * -------------------------------------------------------------------------- */
#include <main.h>
#include <stdbool.h>
#include <stdio.h>


#define TMC5160_COUNT 5U


static GPIO_TypeDef *const tmc_cs_ports[TMC5160_COUNT] = {
    GPIOE, GPIOE, GPIOB, GPIOD, GPIOD
};

static const uint16_t tmc_cs_pins[TMC5160_COUNT] = {
    GPIO_PIN_6,
    GPIO_PIN_3,
    GPIO_PIN_7,
    GPIO_PIN_4,
    GPIO_PIN_15
};

static const char *const tmc_names[TMC5160_COUNT] = {
    "X/s0",
    "Y/s1",
    "Z/s2",
    "E1/s4",
    "E3/s6"
};


/*
 * Keep this intentionally slow while debugging.
 *
 * At 168 MHz, 100 loop iterations plus volatile access and branching gives
 * far more timing margin than the TMC5160 requires. Configuration speed is
 * irrelevant because this is only used for register access.
 */
static inline void tmc_spi_delay(void)
{
    for (volatile uint32_t i = 0; i < 100U; i++)
    {
        __NOP();
    }
}


static void tmc5160_deselect_all(void)
{
    for (uint8_t i = 0; i < TMC5160_COUNT; i++)
    {
        HAL_GPIO_WritePin(
            tmc_cs_ports[i],
            tmc_cs_pins[i],
            GPIO_PIN_SET
        );
    }

    /* SPI mode 3 idle state. */
    HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);

    tmc_spi_delay();
}


/*
 * Transfer one byte in SPI mode 3.
 *
 * Sequence for each bit:
 *   1. Drive clock low: falling edge
 *   2. Present MOSI while clock is low
 *   3. Wait for data setup
 *   4. Drive clock high: rising sampling edge
 *   5. Wait, then sample MISO
 */
static uint8_t tmc_sw_spi_transfer(uint8_t data_out)
{
    uint8_t data_in = 0U;

    for (int bit = 7; bit >= 0; bit--)
    {
        /* Falling edge: slave is allowed to change MISO. */
        HAL_GPIO_WritePin(
            SCLK_GPIO_Port,
            SCLK_Pin,
            GPIO_PIN_RESET
        );

        /* Present the next MOSI bit while SCLK is low. */
        HAL_GPIO_WritePin(
            MOSI_GPIO_Port,
            MOSI_Pin,
            (data_out & (1U << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET
        );

        tmc_spi_delay();

        /* Rising edge: TMC5160 samples MOSI. */
        HAL_GPIO_WritePin(
            SCLK_GPIO_Port,
            SCLK_Pin,
            GPIO_PIN_SET
        );

        tmc_spi_delay();

        /* MCU samples MISO after the rising edge. */
        if (HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin) == GPIO_PIN_SET)
        {
            data_in |= (uint8_t)(1U << bit);
        }

        tmc_spi_delay();
    }

    /* Leave clock in the mode-3 idle state. */
    HAL_GPIO_WritePin(
        SCLK_GPIO_Port,
        SCLK_Pin,
        GPIO_PIN_SET
    );

    return data_in;
}


/*
 * Send one complete 40-bit TMC5160 datagram.
 *
 * The first returned byte is the TMC status byte. The remaining four bytes
 * are the register payload returned by the preceding pipelined request.
 */
static uint32_t tmc5160_transfer_frame(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t address,
    uint32_t tx_data,
    uint8_t *status_out)
{
    uint32_t rx_data = 0U;

    /*
     * Guarantee that no second driver is still selected and satisfy the
     * minimum CS-high time between datagrams.
     */
    tmc5160_deselect_all();

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    tmc_spi_delay();

    uint8_t status = tmc_sw_spi_transfer(address);

    rx_data |= (uint32_t)tmc_sw_spi_transfer(
        (uint8_t)(tx_data >> 24)
    ) << 24;

    rx_data |= (uint32_t)tmc_sw_spi_transfer(
        (uint8_t)(tx_data >> 16)
    ) << 16;

    rx_data |= (uint32_t)tmc_sw_spi_transfer(
        (uint8_t)(tx_data >> 8)
    ) << 8;

    rx_data |= (uint32_t)tmc_sw_spi_transfer(
        (uint8_t)tx_data
    );

    tmc_spi_delay();

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /*
     * The TMC5160 processes the datagram on the CS rising edge. Keep CS high
     * before starting another transaction.
     */
    tmc_spi_delay();
    tmc_spi_delay();

    if (status_out != NULL)
    {
        *status_out = status;
    }

    return rx_data;
}


void tmc5160_write(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t reg,
    uint32_t value)
{
    /*
     * Bit 7 of the first byte:
     *   1 = write
     *   0 = read
     */
    (void)tmc5160_transfer_frame(
        cs_port,
        cs_pin,
        (uint8_t)((reg & 0x7FU) | 0x80U),
        value,
        NULL
    );
}


uint32_t tmc5160_read(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t reg,
    uint8_t *status_out)
{
    uint8_t address = reg & 0x7FU;

    /*
     * TMC reads are pipelined.
     *
     * Frame 1 requests the address. Its returned 32-bit payload belongs to
     * the previous transaction and is discarded.
     */
    (void)tmc5160_transfer_frame(
        cs_port,
        cs_pin,
        address,
        0U,
        NULL
    );

    /*
     * Frame 2 clocks out the value requested by frame 1.
     */
    return tmc5160_transfer_frame(
        cs_port,
        cs_pin,
        address,
        0U,
        status_out
    );
}

static bool tmc5160_write_verified(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t reg,
    uint32_t value,
    uint32_t verify_mask,
    const char *driver_name,
    const char *register_name)
{
    const uint8_t max_attempts = 3U;

    for (uint8_t attempt = 1U; attempt <= max_attempts; attempt++)
    {
        tmc5160_write(cs_port, cs_pin, reg, value);

        /*
         * Allow the CS rising edge to complete register processing before
         * issuing the readback.
         */
        tmc_spi_delay();
        tmc_spi_delay();

        uint32_t readback = tmc5160_read(
            cs_port,
            cs_pin,
            reg,
            NULL
        );

        if ((readback & verify_mask) == (value & verify_mask))
        {
            printf(
                "TMC %-5s %-12s OK: wrote=0x%08lX read=0x%08lX\r\n",
                driver_name,
                register_name,
                (unsigned long)value,
                (unsigned long)readback
            );

            return true;
        }

        printf(
            "TMC %-5s %-12s attempt %u failed: "
            "wrote=0x%08lX read=0x%08lX\r\n",
            driver_name,
            register_name,
            attempt,
            (unsigned long)value,
            (unsigned long)readback
        );

        tmc5160_deselect_all();
    }

    printf(
        "TMC %-5s ERROR: could not write %s\r\n",
        driver_name,
        register_name
    );

    return false;
}

bool ConfigureSPIControllers(void)
{
    bool all_ok = true;

    /*
     * Establish a known bus state before the first transaction.
     */
    tmc5160_deselect_all();

    for (uint8_t i = 0; i < TMC5160_COUNT; i++)
    {
        GPIO_TypeDef *cs_port = tmc_cs_ports[i];
        uint16_t cs_pin = tmc_cs_pins[i];
        const char *name = tmc_names[i];

        /*
         * Confirm that a TMC5160 is actually present before configuring it.
         * IOIN.VERSION should be 0x30.
         */
        uint32_t ioin = tmc5160_read(
            cs_port,
            cs_pin,
            0x04,
            NULL
        );

        uint8_t version = (uint8_t)(ioin >> 24);

        if (version != 0x30U)
        {
            printf(
                "TMC %-5s not present or SPI unavailable: "
                "IOIN=0x%08lX VERSION=0x%02X\r\n",
                name,
                (unsigned long)ioin,
                version
            );

            /*
             * This lets you intentionally remove one driver without treating
             * the absent socket as a fatal write error.
             */
            continue;
        }

        printf(
            "Configuring TMC %-5s, VERSION=0x%02X\r\n",
            name,
            version
        );

        /*
         * CHOPCONF
         *
         * Preserve your requested value:
         *   0x140100C3
         *
         * Verify all writable bits. Some status-like or reserved bits should
         * not be included in the comparison, but this register value should
         * normally read back exactly.
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x6C,
            0x140100C3U,
            0xFFFFFFFFU,
            name,
            "CHOPCONF"
        );

        /*
         * GLOBALSCALER = 0x80 = 128
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x0B,
            0x00000080U,
            0x000000FFU,
            name,
            "GLOBALSCALER"
        );

        /*
         * IHOLD_IRUN
         *
         * Your current diagnostic setting:
         *   IHOLD      = 1
         *   IRUN       = 7
         *   IHOLDDELAY = 6
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x10,
            0x00060A05U,
            0x000FFFFFU,
            name,
            "IHOLD_IRUN"
        );

        /*
         * TPOWERDOWN = 10
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x11,
            0x0000000AU,
            0x000000FFU,
            name,
            "TPOWERDOWN"
        );

        /*
         * PWMCONF
         *
         * Keep the same value even though spreadCycle is selected. It will
         * remain available if stealthChop is enabled later.
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x70,
            0xC40C001EU,
            0xFFFFFFFFU,
            name,
            "PWMCONF"
        );

        /*
         * GCONF = 0x08
         *
         * en_pwm_mode is clear, so spreadCycle is selected.
         * multistep_filt is set.
         */
        all_ok &= tmc5160_write_verified(
            cs_port,
            cs_pin,
            0x00,
            0x00000008U,
            0x0000001FU,
            name,
            "GCONF"
        );

        /*
         * GSTAT is write-one-to-clear. It should not be verified using the
         * normal equality helper because reading it after a successful clear
         * should return zero, not 0x07.
         */
        tmc5160_write(
            cs_port,
            cs_pin,
            0x01,
            0x00000007U
        );

        uint32_t gstat = tmc5160_read(
            cs_port,
            cs_pin,
            0x01,
            NULL
        );

        printf(
            "TMC %-5s GSTAT after clear: 0x%08lX\r\n",
            name,
            (unsigned long)gstat
        );

        if ((gstat & 0x07U) != 0U)
        {
            all_ok = false;

            printf(
                "TMC %-5s WARNING: GSTAT remains set: 0x%08lX\r\n",
                name,
                (unsigned long)gstat
            );
        }
    }

    tmc5160_deselect_all();

    if (all_ok)
    {
        printf("TMC5160 configuration completed successfully.\r\n");
    }
    else
    {
        printf("TMC5160 configuration completed with errors.\r\n");
    }

    return all_ok;
}
