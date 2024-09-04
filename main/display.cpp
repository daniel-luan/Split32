
#include "display.h"

void Display::init_spi()
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(spi_bus_config_t));
    buscfg.mosi_io_num = MOSI_PIN;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = SCLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(DISPLAY_TAG, "SPI BUS Initizlized!");

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.mode = 0;                 // SPI mode 0
    devcfg.clock_speed_hz = 4000000; // Clock out at 4 MHz
    devcfg.spics_io_num = CS_PIN;    // CS pin
    devcfg.queue_size = 7;           // We want to be able to queue 7 transactions at a time
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(DISPLAY_TAG, "SPI Device Added!");
}

void Display::init_gpio()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << DC_PIN) | (1ULL << RST_PIN));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Reset the display
    gpio_set_level(RST_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(RST_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_config_t io_conf2 = {};
    io_conf2.pin_bit_mask = (1ULL << BUSY_PIN);
    io_conf2.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf2);
}

void Display::wait_while_busy(const char *name)
{

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(DISPLAY_TAG, "Wait While Busy: %s", name);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int count = 0;
    while (1)
    {
        count++;
        if (gpio_get_level(BUSY_PIN) != 1)
            break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (count > 30000)
        {
            ESP_LOGI(DISPLAY_TAG, "Wait While Busy Timed out");
            break;
        }
    }
    ESP_LOGI(DISPLAY_TAG, "Wait While Busy Done, took %d ms", count * 10);
}

void Display::write_command(const uint8_t cmd)
{
    ESP_LOGI(DISPLAY_TAG, "Writing CMD: %x", cmd);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is 8 bits
    t.tx_buffer = &cmd;       // The data is the cmd itself

    gpio_set_level(DC_PIN, 0);
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void Display::write_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
    {
        return; // no need to send anything
    }
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = len * 8;       // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;       // Data

    gpio_set_level(DC_PIN, 1);
    ret = spi_device_polling_transmit(spi, &t); // Transmit!

    assert(ret == ESP_OK); // Should have had no issues.
}

void Display::set_partial_ram_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    write_command(0x44);
    write_data(x / 8);
    write_data((x + w - 1) / 8);
    write_command(0x45);
    write_data(y % 256);
    write_data(y / 256);
    write_data((y + h - 1) % 256);
    write_data((y + h - 1) / 256);
    write_command(0x4e);
    write_data(x / 8);
    write_command(0x4f);
    write_data(y % 256);
    write_data(y / 256);
}

void Display::init_display()
{
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms according to specs
    write_command(0x12);                 // SWRESET
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms according to specs
    write_command(0x01);                 // Driver output control
    write_data(0x27);
    write_data(0x01);
    write_data(0x00);
    write_command(0x11); // data entry mode
    write_data(0x03);
    write_command(0x3C); // BorderWavefrom
    write_data(0x05);
    write_command(0x18); // Read built-in temperature sensor
    write_data(0x80);
    write_command(0x21); //  Display update control
    write_data(0x00);
    write_data(0x80);
    set_partial_ram_area(0, 0, WIDTH, HEIGHT);
}

void Display::update_part()
{
    write_command(0x22);
    write_data(0xf7);
    write_command(0x20);
    wait_while_busy("_update_part");
}

void Display::power_on()
{
    if (!_power_is_on)
    {
        write_command(0x22);
        write_data(0xf8);
        write_command(0x20);
        wait_while_busy("_power_on");
    }
    _power_is_on = true;
}

void Display::power_off()
{
    if (_power_is_on)
    {
        write_command(0x22);
        write_data(0x83);
        write_command(0x20);
        wait_while_busy("_power_off");
    }
    _power_is_on = false;
}

void Display::fill_screen(uint8_t bw, uint8_t red)
{
    init_part();
    set_partial_ram_area(0, 0, WIDTH, HEIGHT);
    write_command(0x24);
    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
    {
        write_data(bw);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
    write_command(0x26);

    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
    {
        write_data(red);
    }
}

void Display::refresh()
{
    init_part();
    set_partial_ram_area(0, 0, WIDTH, HEIGHT);
    update_part();
}

void Display::write_display(const uint8_t bw[4736], const uint8_t red[4736])
{
    init_part();
    set_partial_ram_area(0, 0, WIDTH, HEIGHT);
    if (bw)
    {

        write_command(0x24);
        for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
        {
            write_data(bw[i]);
        }
    }

    if (red)
    {
        write_command(0x26);
        for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
        {
            write_data(red[i]);
        }
    }
}
