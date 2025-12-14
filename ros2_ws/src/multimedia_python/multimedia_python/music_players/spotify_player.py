from .base_music_player import BaseMusicPlayer

from ..external import spotipy
from ..external.spotipy.oauth2 import SpotifyOAuth


class SpotifyPlayer(BaseMusicPlayer):

    def __init__(self):
        super().__init__()

        self._preferred_device_name = 'rascupi'

        self._sp = spotipy.Spotify(
            auth_manager=SpotifyOAuth(
                scope='user-read-playback-state user-modify-playback-state user-read-currently-playing')
        )

    def _get_device_id(self) -> str:
        devices = self._sp.devices().get("devices", [])
        if not devices:
            raise RuntimeError(
                "No Spotify devices available. "
                "Start Spotify (desktop/web) or a Spotify Connect client."
            )

        if self._preferred_device_name:
            preferred = self._preferred_device_name.lower()
            for d in devices:
                name = (d.get("name") or "").lower()
                if name == preferred:
                    return d["id"]

        # fallback: first device
        return devices[0]["id"]

    def play(self, frequency: int | None = None, volume: float | None = None) -> None:
        if volume is not None:
            self.set_volume(volume)

        self._sp.start_playback(device_id=self._get_device_id())
        self._is_playing = True

    def stop(self) -> None:
        self._sp.pause_playback(device_id=self._get_device_id())
        self._is_playing = False

    def toggle_play_stop(self, frequency: int | None = None, volume: float | None = None) -> None:
        playback = self._sp.current_playback()
        is_playing = bool(playback and playback.get("is_playing", False))

        if is_playing:
            self.stop()
        else:
            self.play(volume=volume)

    def play_next(self) -> int:
        self._sp.next_track(device_id=self._get_device_id())
        self._is_playing = True
        return -1

    def play_previous(self) -> int:
        self._sp.previous_track(device_id=self._get_device_id())
        self._is_playing = True
        return -1

    def set_volume(self, volume: float) -> float:
        self._sp.volume(volume_percent=self._convert_volume(
            volume), device_id=self._get_device_id())
        return volume

    def set_frequency(self, frequency: int) -> int:
        return -1

    def get_header_text(self) -> str:
        is_playing_text: str = 'Playing' if self._is_playing else 'Stopped'
        return f'Spotify - {is_playing_text}'

    def get_info_text(self) -> str:
        current = self._sp.currently_playing()
        if not current or not current.get('item'):
            self._is_playing = False
            return 'No track playing'

        item = current['item']
        artists = ", ".join(a['name'] for a in item['artists'])
        title = item['name']

        self._is_playing = bool(current.get('is_playing', False))
        return f'{artists} - {title}'
