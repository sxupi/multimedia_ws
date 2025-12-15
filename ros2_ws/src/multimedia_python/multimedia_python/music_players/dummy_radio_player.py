from .base_music_player import BaseMusicPlayer
import random


class DummyRadioPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

        # Set new volume ranges
        self._min_volume = 0
        self._max_volume = 15

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
        return random.randint(880, 1060)

    def play_previous(self) -> int:
        return random.randint(880, 1060)

    def set_volume(self, volume: float) -> float:
        self._is_playing = volume > 0.02
        converted_volume = self._convert_volume(volume)
        return volume

    def set_frequency(self, frequency: int) -> int:
        return frequency

    def get_header_text(self) -> str:
        is_playing_text: str = 'Playing' if self._is_playing else 'Stopped'
        return f'Radio - {is_playing_text}'

    def get_info_text(self) -> str:
        return 'Test radio text'
