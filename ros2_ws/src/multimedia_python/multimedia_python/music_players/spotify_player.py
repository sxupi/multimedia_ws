from abc import abstractmethod

from .base_music_player import BaseMusicPlayer

class SpotifyPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

    def play(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def toggle_play_stop(self) -> tuple[str, str]:
        pass

    def play_next(self) -> tuple[str, str, int]:
        pass

    def play_previous(self) -> tuple[str, str, int]:
        pass

    def set_volume(self, volume: float) -> int:
        pass

    def set_frequency(self, frequency: int) -> int:
        return -1