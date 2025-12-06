#include "multimedia_cpp/si470x/si470x.hpp"

#include <linux/i2c-dev.h>
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>

Si470x::Si470x(const std::string &i2c_dev, int addr, int reset_gpio)
  : fd_(-1),
    addr_(addr),
    reset_gpio_(reset_gpio)
{
    fd_ = ::open(i2c_dev.c_str(), O_RDWR);
    if (fd_ < 0)
    {
        std::perror("open i2c");
        return;
    }
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0)
    {
        std::perror("ioctl I2C_SLAVE");
        ::close(fd_);
        fd_ = -1;
    }
}

Si470x::~Si470x()
{
    if (fd_ >= 0)
        ::close(fd_);
}

bool Si470x::reset_chip()
{
    gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip)
    {
        std::perror("gpiod_chip_open_by_name");
        return false;
    }

    gpiod_line *rst = gpiod_chip_get_line(chip, reset_gpio_);
    if (!rst)
    {
        std::perror("gpiod_chip_get_line");
        gpiod_chip_close(chip);
        return false;
    }

    if (gpiod_line_request_output(rst, "si470x_rst", 0) < 0)
    {
        std::perror("gpiod_line_request_output");
        gpiod_chip_close(chip);
        return false;
    }

    using namespace std::chrono_literals;
    gpiod_line_set_value(rst, 0);
    std::this_thread::sleep_for(10ms);
    gpiod_line_set_value(rst, 1);
    std::this_thread::sleep_for(100ms);

    gpiod_line_release(rst);
    gpiod_chip_close(chip);
    return true;
}

bool Si470x::read_registers(uint16_t *regs, int count)
{
    if (fd_ < 0) return false;

    uint8_t buf[32];
    if (::read(fd_, buf, sizeof(buf)) != (int)sizeof(buf))
    {
        std::perror("si470x read");
        return false;
    }

    for (int i = 0; i < count; ++i)
        regs[i] = (buf[2*i] << 8) | buf[2*i + 1];

    return true;
}

bool Si470x::write_registers(uint16_t *regs, int count)
{
    if (fd_ < 0) return false;

    uint8_t buf[32];
    for (int i = 0; i < count; ++i)
    {
        buf[2*i]     = (regs[i] >> 8) & 0xFF;
        buf[2*i + 1] = regs[i] & 0xFF;
    }

    if (::write(fd_, buf, sizeof(buf)) != (int)sizeof(buf))
    {
        std::perror("si470x write");
        return false;
    }
    return true;
}

bool Si470x::init()
{
    if (fd_ < 0) return false;

    if (!reset_chip())
        return false;

    uint16_t regs[16] = {0};
    if (!read_registers(regs))
        return false;

    // Very basic power-up configuration.
    // For a production system, refer to the Si470x datasheet or a full driver.
    // Here we just set POWERCFG (0x02) to power up.
    regs[0x02] |= 0x4001;  // ENABLE + POWERUP bits (simplified)
    if (!write_registers(regs))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    return true;
}

bool Si470x::set_frequency_10k(int ch10)
{
    if (fd_ < 0) return false;

    float freq_mhz = ch10 / 10.0f;
    // Assume EU band 87.5–108MHz, 100kHz steps
    int channel = static_cast<int>((freq_mhz - 87.5f) / 0.1f);
    if (channel < 0) channel = 0;
    if (channel > 0x03FF) channel = 0x03FF; // 10 bits

    uint16_t regs[16] = {0};
    if (!read_registers(regs))
        return false;

    // CHANNEL register often at 0x03
    regs[0x03] &= 0xFE00;          // clear channel bits
    regs[0x03] |= (channel & 0x03FF);
    regs[0x03] |= (1u << 15);      // TUNE bit

    if (!write_registers(regs))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}