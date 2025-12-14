import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

from enum import Enum

from .music_players.base_music_player import BaseMusicPlayer
from .music_players.radio_player import RadioPlayer
from .music_players.spotify_player import SpotifyPlayer


class MusicPlayerEnum(Enum):
    RADIO = 'Radio'
    SPOTIFY = 'Spotify'


class MusicControllerNode(Node):

    def __init__(self):
        super().__init__('music_controller')

        self._volume_subscriber_ = self.create_subscription(
            Float32,
            '/current/volume_float32',
            self.__volume_change_callback,
            10
        )
        self._frequency_subscriber_ = self.create_subscription(
            Int32,
            '/current/frequency_int32',
            self.__frequency_change_callback,
            10
        )
        self._command_subscriber_ = self.create_subscription(
            String,
            '/remote/command_string',
            self.__incoming_command_callback,
            10
        )

        self._frequency_publisher_ = self.create_publisher(
            Int32,
            '/current/frequency_int32',
            10,
        )
        self._header_publisher_ = self.create_publisher(
            String,
            '/display/header_string',
            10
        )
        self._info_publisher_ = self.create_publisher(
            String,
            '/display/text_string',
            10
        )

        # Current information
        self._curr_freq = 900
        self._curr_volume = 0.0

        # Initailize both players (without playing anything)
        self._radio_player: RadioPlayer = RadioPlayer()
        self._spotify_player: SpotifyPlayer = SpotifyPlayer()

        # Set the current player
        self._curr_player: BaseMusicPlayer = self._radio_player
        self._curr_player_tag: MusicPlayerEnum = MusicPlayerEnum.RADIO

        self._header_timer = self.create_timer(1.0, self.__refresh_header)
        self._info_timer = self.create_timer(0.5, self.__refresh_info_text)

    def __volume_change_callback(self, msg: Float32) -> None:
        new_volume = -1
        try:
            new_volume = self._curr_player.set_volume(msg.data)
        except OSError as ose:
            self.get_logger().error(f'Device error setting volume: {ose}')
            return
        except Exception as e:
            self.get_logger().error(f'Something went wrong: {e}')
            return
        self._curr_volume = new_volume
        self.get_logger().info(f'Set volume to {new_volume}')

    def __frequency_change_callback(self, msg: Int32) -> None:
        if self._curr_freq == msg.data:
            return

        # Always store the user's knob intent
        self._curr_freq = int(msg.data)

        try:
            tuned = self._curr_player.set_frequency(self._curr_freq)
        except OSError as ose:
            self.get_logger().error(f'Device error setting frequency: {ose}')
            return
        except Exception as e:
            self.get_logger().error(f'Something went wrong: {e}')
            return

        if tuned == -1:
            self.get_logger().debug('Active player ignored frequency')
        else:
            # if RadioPlayer clamps/quantizes it, sync to actual tuned value
            self._curr_freq = tuned
            self.get_logger().info(f'Set frequency to {tuned}')

    def __incoming_command_callback(self, msg: String) -> None:
        match msg.data:
            case 'PLAY_STOP':
                self._handle_play_stop()
                return
            case 'SWITCH':
                self.__handle_switch()
                return
            case 'NEXT':
                self.__handle_change(True)
                return
            case 'PREV':
                self.__handle_change(False)
                return

    def _handle_play_stop(self) -> None:
        try:
            self._curr_player.toggle_play_stop(
                self._curr_freq, self._curr_volume)
        except OSError as ose:
            self.get_logger().error(f'Device error toggling play/stop: {ose}')
            return
        except Exception as e:
            self.get_logger().error(
                f'Something went wrong toggling play/stop: {e}')
            return

    def __handle_switch(self) -> None:
        try:
            self._curr_player.stop()
        except OSError as ose:
            self.get_logger().error(f'Device error setting volume: {ose}')
            return
        except Exception as e:
            self.get_logger().error(f'Something went wrong: {e}')
            return

        if self._curr_player_tag == MusicPlayerEnum.RADIO:
            self._curr_player_tag = MusicPlayerEnum.SPOTIFY
            self._curr_player = self._spotify_player
        else:
            self._curr_player_tag = MusicPlayerEnum.RADIO
            self._curr_player = self._radio_player

        try:
            self._curr_player.play(self._curr_freq, self._curr_volume)
        except OSError as ose:
            self.get_logger().error(f'Device error setting volume: {ose}')
            return
        except Exception as e:
            self.get_logger().error(f'Something went wrong: {e}')
            return

    def __handle_change(self, is_next: bool):
        freq = -1
        try:
            freq = self._curr_player.play_next() if is_next else self._curr_player.play_previous()
        except OSError as ose:
            self.get_logger().error(f'Device error during seek down: {ose}')
            return
        except Exception as e:
            self.get_logger().error(f'Something went wrong: {e}')
            return

        if freq != -1:
            self._curr_freq = freq
            msg: Int32 = Int32()
            msg.data = self._curr_freq
            self._frequency_publisher_.publish(msg)
            self.get_logger().info(
                f'New channel at {self._curr_freq} set and published')
        else:
            self.get_logger().info('Next/previous song set')

    def __refresh_header(self):
        msg: String = String()
        msg.data = self._curr_player.get_header_text()
        self._header_publisher_.publish(msg)

    def __refresh_info_text(self):
        info_text: str = ''
        try:
            info_text = self._curr_player.get_info_text()
        except OSError as ose:
            self.get_logger().error(
                f'Device error during getting info text: {ose}')
            return
        except Exception as e:
            self.get_logger().error(
                f'Something went wrong during getting info text: {e}')
            return

        msg: String = String()
        msg.data = info_text if info_text else 'No current info'
        self._info_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MusicControllerNode()
    node.get_logger().info('Starting to spin the music controller node now')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
