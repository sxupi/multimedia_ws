#!/usr/bin/env python3
import time
from typing import Optional

from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.lcd.device import st7735
from PIL import Image, ImageDraw, ImageFont


class TwoLineDisplay:
    """
    Helper for a 160x128 ST7735 display with two lines:
    - display_header(text): first (top) line
    - display_text(text):   second (bottom) line

    Both lines scroll horizontally if they don't fit.
    Call update() regularly to animate.
    """

    def __init__(
        self,
        device,
        font_path: Optional[str] = None,
        font_size: int = 12,
        scroll_speed_px: int = 1,
        scroll_step_time: float = 0.05,
        padding: int = 4,
        header_y: int = 20,
        text_y: int = 70,
    ):
        self.device = device
        self.padding = padding
        self.scroll_speed_px = scroll_speed_px
        self.scroll_step_time = scroll_step_time
        self.header_y = header_y
        self.text_y = text_y

        # Load font
        if font_path:
            self.font = ImageFont.truetype(font_path, font_size)
        else:
            self.font = ImageFont.load_default()

        # Text content
        self._header_text = ""
        self._text_line = ""

        # Widths in pixels
        self._header_width = 0
        self._text_width = 0

        # Scroll offsets
        self._header_offset = 0
        self._text_offset = 0

        # Timing
        self._last_update = 0.0

        # Gap between repetitions in marquee mode
        self._gap_px = 20

    # ---------- public API ----------

    def display_header(self, text: str) -> None:
        """Set first line text (station name)."""
        self._header_text = text or ""
        self._header_width = self._measure_width(self._header_text)
        self._header_offset = 0

    def display_text(self, text: str) -> None:
        """Set second line text (info / RDS)."""
        self._text_line = text or ""
        self._text_width = self._measure_width(self._text_line)
        self._text_offset = 0

    def update(self, now: Optional[float] = None) -> None:
        """
        Redraw and scroll if needed.
        Call this regularly: e.g. every 10–50 ms.
        """
        if now is None:
            now = time.time()

        # Only move text if enough time passed
        if (now - self._last_update) >= self.scroll_step_time:
            self._advance_offsets()
            self._last_update = now

        self._draw()

    # ---------- internal helpers ----------

    def _measure_width(self, text: str) -> int:
        if not text:
            return 0

        # Newer Pillow (>=10) prefers getlength / getbbox, getsize is removed
        try:
            # Pillow 8+ has getlength for text width
            width = self.font.getlength(text)
            return int(width)
        except AttributeError:
            # Fallback path: use a temporary ImageDraw to measure
            img = Image.new("RGB", (1, 1))
            draw = ImageDraw.Draw(img)
            w, _ = draw.textsize(text, font=self.font)
            return int(w)

    def _should_scroll(self, width_px: int, line_width_px: int) -> bool:
        return width_px > line_width_px

    def _advance_offsets(self) -> None:
        width = self.device.width
        line_width = width - 2 * self.padding

        # Header line
        if self._should_scroll(self._header_width, line_width):
            self._header_offset += self.scroll_speed_px
            total = self._header_width + self._gap_px
            if self._header_offset >= total:
                self._header_offset = 0

        # Second text line
        if self._should_scroll(self._text_width, line_width):
            self._text_offset += self.scroll_speed_px
            total = self._text_width + self._gap_px
            if self._text_offset >= total:
                self._text_offset = 0

    def _draw(self) -> None:
        width = self.device.width
        height = self.device.height
        line_width = width - 2 * self.padding

        with canvas(self.device) as draw:
            # Background
            draw.rectangle((0, 0, width, height),
                           outline="white", fill="black")

            # ----- header line (top) -----
            if self._header_text:
                if self._should_scroll(self._header_width, line_width):
                    x_base = self.padding - self._header_offset
                    # first copy
                    draw.text(
                        (x_base, self.header_y),
                        self._header_text,
                        font=self.font,
                        fill="white",
                    )
                    # wrap copy
                    x2 = x_base + self._header_width + self._gap_px
                    draw.text(
                        (x2, self.header_y),
                        self._header_text,
                        font=self.font,
                        fill="white",
                    )
                else:
                    # centered
                    x = (width - self._header_width) // 2
                    draw.text(
                        (x, self.header_y),
                        self._header_text,
                        font=self.font,
                        fill="white",
                    )

            # ----- second line (bottom) -----
            if self._text_line:
                if self._should_scroll(self._text_width, line_width):
                    x_base = self.padding - self._text_offset
                    draw.text(
                        (x_base, self.text_y),
                        self._text_line,
                        font=self.font,
                        fill="white",
                    )
                    x2 = x_base + self._text_width + self._gap_px
                    draw.text(
                        (x2, self.text_y),
                        self._text_line,
                        font=self.font,
                        fill="white",
                    )
                else:
                    x = (width - self._text_width) // 2
                    draw.text(
                        (x, self.text_y),
                        self._text_line,
                        font=self.font,
                        fill="white",
                    )


def create_device() -> st7735:
    """
    Create the ST7735 device for your wiring.
    Adjust rotate/bgr if your orientation/colors differ.
    """
    serial = spi(
        port=0,
        device=0,
        gpio_DC=25,   # GPIO25 -> DC
        gpio_RST=24,  # GPIO24 -> RST
        bus_speed_hz=4_000_000,
    )

    device = st7735(
        serial_interface=serial,
        width=160,
        height=128,
        rotate=0,     # 0/1/2/3 depending on how your TFT is mounted
        bgr=True,
    )
    return device


if __name__ == "__main__":
    # Simple demo without ROS2
    dev = create_device()
    disp = TwoLineDisplay(dev)

    disp.display_header("FM 90.3 Super Long Station Name That Scrolls")
    disp.display_text(
        "Now playing: An extremely long song title that will scroll nicely.")

    try:
        while True:
            disp.update()
            time.sleep(0.02)  # 20 ms -> ~50 FPS scrolling
    except KeyboardInterrupt:
        pass
