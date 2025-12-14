from abc import abstractmethod


class BaseMusicPlayer():

    def __init__(self):
        self._is_playing = False
        
        # Default values (can be overwritten in the specific players)
        self._min_volume = 0
        self._max_volume = 100

    def __convert_volume(self) -> int:
        # Clamp 0.0–1.0 to MIN–MAX
        if volume < 0.0:
            volume = 0.0
        if volume > 1.0:
            volume = 1.0
        return int(self._min_volume + volume * (self._max_volume - self._min_volume))

    @abstractmethod
    def play(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def toggle_play_stop(self) -> None:
        pass

    @abstractmethod
    def play_next(self) -> int:
        pass

    @abstractmethod
    def play_previous(self) -> int:
        pass

    @abstractmethod
    def set_volume(self) -> int:
        pass

    @abstractmethod
    def set_frequency(self, frequency: int) -> int:
        pass

    @abstractmethod
    def get_header_text(self) -> str:
        pass

    @abstractmethod
    def get_info_text(self) -> str:
        pass