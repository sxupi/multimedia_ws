from ..external.si470x import Si4703Radio

from .base_music_player import BaseMusicPlayer


class RadioPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

        # Set new volume ranges
        self._min_volume = 0
        self._max_volume = 15

        # Initalize radio driver
        self._radio = Si4703Radio(i2c_addr=0x10, reset_pin=23)

        # Set initial state on hardware
        self.set_frequency(900)
        self.set_volume(0.1)

    def play(self, frequency: int | None = None, volume: float | None = None) -> None:
        if frequency is not None:
            self.set_frequency(frequency)
        if volume is not None:
            self.set_volume(volume)

    def stop(self) -> None:
        self.set_volume(0.0)

    def toggle_play_stop(self, frequency: int | None = None, volume: float | None = None) -> None:
        if self._is_playing:
            self.stop()
        else:
            self.play(frequency, volume)

    def play_next(self) -> int:
        self._radio.si4703SeekUp()
        return self._radio.si4703GetChannel()

    def play_previous(self) -> int:
        self._radio.si4703SeekDown()
        return self._radio.si4703GetChannel()

    def set_volume(self, volume: float) -> float:
        self._is_playing = volume > 0.02
        converted_volume = self._convert_volume(volume)
        self._radio.si4703SetVolume(converted_volume)
        return volume

    def set_frequency(self, frequency: int) -> int:
        self._radio.si4703SetChannel(frequency)
        return frequency

    def get_header_text(self) -> str:
        is_playing_text: str = 'Playing' if self._is_playing else 'Stopped'
        return f'Radio - {is_playing_text}'

    def get_info_text(self) -> str:
        self._radio.si4703ClearRDSBuffers()
        self._radio.si4703ProcessRDS()
        radio_text = self._radio.si4703GetProgramService()
        return radio_text if radio_text else 'No text'
