#include "multimedia_cpp/lcd_1602/lcd_1602.hpp"

Lcd1602Display::Lcd1602Display(uint8_t addr, uint8_t cols, uint8_t rows)
  : lcd_(addr, cols, rows),
    cols_(cols),
    rows_(rows)
{
}

bool Lcd1602Display::init()
{
    // Library API: init, backlight, clear
    lcd_.init();
    lcd_.backlight();
    lcd_.clear();
    return true;
}

void Lcd1602Display::clear()
{
    lcd_.clear();
}

void Lcd1602Display::print_line(uint8_t row, const std::string &text)
{
    std::string s = text;
    if (s.size() < cols_)
    {
        s.append(cols_ - s.size(), ' ');
    }
    else if (s.size() > cols_)
    {
        s = s.substr(0, cols_);
    }

    lcd_.setCursor(0, row);
    lcd_.print(s.c_str());
}