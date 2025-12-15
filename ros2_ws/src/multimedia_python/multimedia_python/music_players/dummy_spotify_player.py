from .base_music_player import BaseMusicPlayer
import random

class DummySpotifyPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

    def play(self, frequency: int | None = None, volume: float | None = None) -> None:
        if volume is not None:
            self.set_volume(volume)
        self._is_playing = True

    def stop(self) -> None:
        self._is_playing = False

    def toggle_play_stop(self, frequency: int | None = None, volume: float | None = None) -> None:
        is_playing = random.randint(0, 100) > 50

        if is_playing:
            self.stop()
        else:
            self.play(volume=volume)

    def play_next(self) -> int:
        return -1

    def play_previous(self) -> int:
        return -1

    def set_volume(self, volume: float) -> float:
        return volume

    def set_frequency(self, frequency: int) -> int:
        return -1

    def get_header_text(self) -> str:
        is_playing_text: str = 'Playing' if self._is_playing else 'Stopped'
        return f'Spotify - {is_playing_text}'

    def get_info_text(self) -> str:
        return 'Artist - Songname'
