from .base_music_player import BaseMusicPlayer


class SpotifyPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

    def play(self, frequency: int | None = None, volume: float | None = None) -> None:
        pass

    def stop(self) -> None:
        pass

    def toggle_play_stop(self, frequency: int | None = None, volume: float | None = None) -> None:
        pass

    def play_next(self) -> int:
        pass

    def play_previous(self) -> int:
        pass

    def set_volume(self, volume: float) -> float:
        pass

    def set_frequency(self, frequency: int) -> int:
        return -1

    def get_header_text(self) -> str:
        is_playing_text: str = 'Playing' if self._is_playing else 'Stopped'
        return f'Spotify - {is_playing_text}'

    def get_info_text(self) -> str:
        pass
