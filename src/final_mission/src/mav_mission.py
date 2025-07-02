# mav_mission.py
from dataclasses import dataclass
from typing     import List, Callable
import math
from pymavlink  import mavutil
import geometry_utils as gu        # calculate_loiter_center, EARTH_RADIUS

# ────────────────────────────────────────────────────────────────
@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float
    action:     str | None = None   # 'loiter', 'land', ...
    transition: str | None = None   # 'transition_fw', 'transition_mc'

# ────────────────────────────────────────────────────────────────
class MissionUploader:
    """
    MAVLink 미션 업로더.
    • PX4 v1.14+에서 테스트
    • 모든 좌표: WGS-84  (deg, m AGL)
    """

    def __init__(self,
                 master: mavutil.mavfile,
                 *,
                 loiter_time:   float,
                 loiter_radius: float,
                 logger: Callable[[str, bool], None] | None = None):
        self.m              = master
        self.loiter_time    = loiter_time
        self.loiter_radius  = loiter_radius
        self.log            = logger or self._stdout_log   # 기본: print

    # ─────────── public ───────────
    def upload(self, wps: List[Waypoint]) -> bool:
        self.log("Starting mission upload…")
        if not self._wait_heartbeat():          return False
        if not self._clear_remote_mission():    return False
        items = self._build_items(wps)
        return self._send_items(items)

    # ─────────── internal: build ───────────
    def _build_items(self, wps: List[Waypoint]) -> list:
        items: list = []
        for i, wp in enumerate(wps):
            # (a) 일반 WAYPOINT
            items.append(self._mk_wp(wp, seq=len(items)))

            # (b) loiter 확장
            if wp.action == 'loiter' and i > 0:
                nxt, prev = wps[i + 1], wps[i - 1]
                loiter_alt = 0.25 * wp.alt + 0.75 * nxt.alt
                center = gu.calculate_loiter_center(prev.__dict__,
                                                    wp.__dict__,
                                                    self.loiter_radius)

                # loiter-time
                items.append(self._encode(len(items),
                                           mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                                           p1=self.loiter_time,
                                           p3=self.loiter_radius,
                                           p4=1,
                                           lat=center['lat'],
                                           lon=center['lon'],
                                           alt=loiter_alt))
                # 원래 WP 복귀
                items.append(self._mk_wp(wp, seq=len(items), alt=loiter_alt))

        # (c) 마지막 LAND
        last = wps[-1]
        items.append(self._encode(len(items),
                                  mavutil.mavlink.MAV_CMD_NAV_LAND,
                                  lat=last.lat, lon=last.lon, alt=last.alt))
        return items

    # ─────────── internal: send ───────────
    def _send_items(self, items: list) -> bool:
        self.log(f"Sending mission item count: {len(items)}")
        self.m.mav.mission_count_send(self.m.target_system,
                                      self.m.target_component,
                                      len(items))

        for _ in items:
            req = self._wait_msg(['MISSION_REQUEST_INT', 'MISSION_REQUEST'])
            if not req:
                self.log("Timeout waiting MISSION_REQUEST", error=True)
                return False
            seq = req.seq
            self.log(f" TX→ item {seq}")
            self.m.mav.send(items[seq])

        ack = self._wait_msg('MISSION_ACK')
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            self.log("Mission uploaded successfully")
            return True
        self.log(f"Mission upload failed: {ack.type if ack else 'NO_ACK'}",
                 error=True)
        return False

    # ─────────── helpers ───────────
    def _mk_wp(self, wp: Waypoint, *, seq: int, alt: float | None = None):
        return self._encode(seq,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            lat=wp.lat, lon=wp.lon, alt=alt or wp.alt)

    def _encode(self, seq: int, cmd: int,
                *, p1=0, p2=0, p3=0, p4=float('nan'),
                lat=float('nan'), lon=float('nan'), alt=float('nan')):
        def to_i32(x: float):  # deg → 1e7 int
            return int(x * 1e7) if not math.isnan(x) else 0

        return self.m.mav.mission_item_int_encode(
            self.m.target_system, self.m.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd, 0, 1,                 # current, autocontinue
            p1, p2, p3, p4,
            to_i32(lat), to_i32(lon), alt
        )

    def _wait_msg(self, types, timeout=10):
        return self.m.recv_match(type=types, blocking=True, timeout=timeout)

    def _wait_heartbeat(self):
        self.log("Waiting for heartbeat…")
        try:
            self.m.wait_heartbeat(timeout=10)
            self.log("Heartbeat received")
            return True
        except mavutil.mavlink.MAVError:
            self.log("No heartbeat", error=True)
            return False

    def _clear_remote_mission(self):
        self.log("Clearing existing mission…")
        self.m.mav.mission_clear_all_send(
            self.m.target_system, self.m.target_component)
        if not self._wait_msg('MISSION_ACK'):
            self.log("No ACK for mission clear", error=True)
            return False
        self.log("Existing mission cleared")
        return True

    # 기본 로그 함수
    @staticmethod
    def _stdout_log(msg: str, error: bool = False):
        print(f"[ERR] {msg}" if error else msg)
