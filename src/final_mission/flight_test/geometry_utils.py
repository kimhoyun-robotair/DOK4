#!/usr/bin/env python3
import math
EARTH_RADIUS = 6371000.0  # 지구 반경 (미터)

# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡNED's Util Funcㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
def wgs84_to_ned(lat, lon, ref_lat, ref_lon):
    """
    주어진 위도와 경도(degrees)를 기준점에 대해 Azimuthal Equidistant Projection 방식으로 투영하여
    로컬 좌표 (x, y) 를 미터 단위로 계산
    """
    # 입력 좌표를 라디안으로 변환
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    ref_sin_lat = math.sin(ref_lat)
    ref_cos_lat = math.cos(ref_lat)
    cos_d_lon = math.cos(lon_rad - ref_lon)

    # 내적 값 계산 후 -1~1 범위로 제한
    arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon
    arg = max(min(arg, 1.0), -1.0)
    c = math.acos(arg)

    # c가 0이 아니면 비율 k 계산 (c/sin(c))
    k = c / math.sin(c) if abs(c) > 1e-6 else 1.0

    x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * EARTH_RADIUS
    y = k * cos_lat * math.sin(lon_rad - ref_lon) * EARTH_RADIUS

    return x, y

def get_distance_between_ned(x_now, y_now, z_now, x_next, y_next, z_next):
    # NED 좌표로 주어진 두 점 사이의 거리 구하기
    dx = x_next - x_now
    dy = y_next - y_now
    dz = z_next - z_now

    dist_xy = math.sqrt(dx*dx+dy*dy)
    dist_z = abs(dz)
    total_dist = math.sqrt(dist_xy*dist_xy + dist_z*dist_z)

    return total_dist, dist_xy, dist_z

def is_waypoint_reached(x_now, y_now, z_now, waypoint, threshold_range):
    # NED 좌표를 기반으로 계산했을 때 경로점에 도착했는지 여부를 판단하기
    total_dist, dist_xy, dist_z = get_distance_between_ned(
        x_now, y_now, z_now,
        waypoint["x"], waypoint["y"], waypoint["z"])
    
    if dist_xy <= threshold_range and dist_z <= threshold_range:
        return True
    else:
        return False
    
def is_height_reached(z_now, z_target, threshold_height):
    # NED 좌표를 기반으로 계산했을 때 경로점의 고도에 도달했는지 여부를 판단하기
    dist_z = abs(z_now - z_target)
    
    if dist_z <= threshold_height:
        return True
    else:
        return False

def get_attitude(current_wp, next_wp):
    # NED 좌표로 주어진 두 점 사이의 yaw 값 계산하기 (제어에 사용)
    dx = next_wp["x"] - current_wp["x"]
    dy = next_wp["y"] - current_wp["y"]

    # arctan2는 (dy, dx)를 인자로 받아 라디안 단위의 각도를 반환.
    yaw_rad = math.atan2(dy, dx)

    # 라디안을 도 단위로 변환
    yaw_deg = math.degrees(yaw_rad)
    return yaw_deg

# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡWGS's Util Funcㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
def calculate_bearing(lat1, lon1, lat2, lon2):
    # WGS84 좌표를 기반으로, 두 점 사이의 yaw값을 계산하기 (제어 등에 사용)
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(y, x)
    # Convert to degrees
    initial_bearing = math.degrees(initial_bearing)
    # Normalize to 0-360
    bearing = (initial_bearing + 360) % 360
    return math.radians(bearing)

def get_distance_to_point_global_wgs84(lat_now, lon_now, lat_next, lon_next):
    # 현재 WGS84 좌표와
    # 다음 경로점까지의 WGS84 경로점 좌표를 받아서
    # 두 점 사이의 거리를 WGS84 좌표 기반으로 계산해서 출력 : 단, 고도는 고려하지 않음

    # 위도와 경도를 라디안 단위로 변환
    current_x_rad = math.radians(lat_next)
    current_y_rad = math.radians(lon_next)
    x_rad = math.radians(lat_now)
    y_rad = math.radians(lon_now)

    # 위도 및 경도 차이를 라디안 단위로 계산
    d_lat = x_rad - current_x_rad
    d_lon = y_rad - current_y_rad

    # 하버사인 공식을 이용해 두 지점 사이의 중심각(c)을 계산
    a = math.sin(d_lat / 2.0) ** 2 + math.sin(d_lon / 2.0) ** 2 * math.cos(current_x_rad) * math.cos(x_rad)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # 수평거리 (dxy, 미터 단위) 계산
    dxy = EARTH_RADIUS * c

    # 각각의 절대값 계산
    dist_xy = abs(dxy)

    return dist_xy

def is_waypoint_reached_wgs84(current_position, waypoint, threshold_range, threshold_height):
    # Calculate the great-circle distance between two points on the Earth's surface
    R = 6371000  # Earth's radius in meters
    lat1, lon1 = map(math.radians, [current_position['lat'], current_position['lon']])
    lat2, lon2 = map(math.radians, [waypoint['lat'], waypoint['lon']])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    horizontal_distance = R * c

    # Calculate vertical distance
    vertical_distance = abs(current_position['alt'] - waypoint['alt'])

    # Check if both horizontal and vertical distances are within threshold
    return (horizontal_distance <= threshold_range and
            vertical_distance <= threshold_height)

def calculate_loiter_center(wp0, wp1, loiter_radius):
    # Convert lat/lon to radians
    lat0, lon0 = map(math.radians, [wp0['lat'], wp0['lon']])
    lat1, lon1 = map(math.radians, [wp1['lat'], wp1['lon']])

    # Calculate bearing from wp0 to wp1
    bearing = math.atan2(
        math.sin(lon1-lon0) * math.cos(lat1),
        math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(lon1-lon0)
    )

    # Calculate perpendicular bearing (add 90 degrees)
    perp_bearing = bearing + math.pi/2

    # Calculate the center point of the loiter circle
    lat_center = math.asin(
        math.sin(lat1) * math.cos(loiter_radius/EARTH_RADIUS) +
        math.cos(lat1) * math.sin(loiter_radius/EARTH_RADIUS) * math.cos(perp_bearing)
    )
    lon_center = lon1 + math.atan2(
        math.sin(perp_bearing) * math.sin(loiter_radius/EARTH_RADIUS) * math.cos(lat1),
        math.cos(loiter_radius/EARTH_RADIUS) - math.sin(lat1) * math.sin(lat_center)
    )

    # Convert back to degrees
    lat_center, lon_center = map(math.degrees, [lat_center, lon_center])

    return {'lat': lat_center, 'lon': lon_center}

def main():
    print("Test")

if __name__=="__main__":
    main()
