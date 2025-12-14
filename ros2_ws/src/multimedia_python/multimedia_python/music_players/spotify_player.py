from abc import abstractmethod

from .base_music_player import BaseMusicPlayer

class SpotifyPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

    def play(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def toggle_play_stop(self) -> None:
        pass

    def play_next(self) -> int:
        pass

    def play_previous(self) -> int:
        pass

    def set_volume(self) -> int:
        pass

    def set_frequency(self, frequency: int) -> int:
        return -1
    
    def get_header_text(self) -> str:
        return 'Spotify'

    def get_info_text(self) -> str:
        pass