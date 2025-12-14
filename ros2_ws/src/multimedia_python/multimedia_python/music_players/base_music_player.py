from abc import ABC, abstractmethod


class BaseMusicPlayer(ABC):

    def __init__(self):
        self._is_playing = False

        # Default values (can be overwritten in the specific players)
        self._min_volume = 0
        self._max_volume = 100

    def _convert_volume(self, volume: float) -> int:
        volume = 0.0 if volume < 0.0 else 1.0 if volume > 1.0 else volume
        return int(round(self._min_volume + volume * (self._max_volume - self._min_volume)))

    @abstractmethod
    def play(self, frequency: int | None = None, volume: float | None = None) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    @abstractmethod
    def toggle_play_stop(self, frequency: int | None = None, volume: float | None = None) -> None:
        ...

    @abstractmethod
    def play_next(self) -> int:
        ...

    @abstractmethod
    def play_previous(self) -> int:
        ...

    @abstractmethod
    def set_volume(self, volume: float) -> float:
        ...

    @abstractmethod
    def set_frequency(self, frequency: int) -> int:
        ...

    @abstractmethod
    def get_header_text(self) -> str:
        ...

    @abstractmethod
    def get_info_text(self) -> str:
        ...
