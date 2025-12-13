from abc import abstractmethod

from ros2_ws.src.multimedia_python.multimedia_python.external.si470x import Si4703Radio

from .base_music_player import BaseMusicPlayer


class RadioPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

        # Initalize radio driver
        self._radio = Si4703Radio(i2c_addr=0x10, reset_pin=23)

        # Set new volume ranges
        self._min_volume = 0
        self._max_volume = 15

        # Set initial state on hardware
        try:
            self._radio.si4703SetChannel(900)
            self._radio.si4703SetVolume(self.__convert_volume(0.0))
        except OSError as e:
            self.get_logger().error(f'I2C error during radio init: {e}')

    def play(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def toggle_play_stop(self) -> None:
        pass

    def play_next(self) -> int:
        self._radio.si4703SeekUp()
        return self._radio.si4703GetChannel()

    def play_previous(self) -> int:
        self._radio.si4703SeekDown()
        return self._radio.si4703GetChannel()

    def set_volume(self, volume: float) -> int:
        converted_volume = self.__convert_volume(volume)
        self._radio.si4703SetVolume(self.__convert_volume(volume))
        return converted_volume

    def set_frequency(self, frequency: int) -> int:
        self._radio.si4703SetChannel(frequency)
        return frequency
