import time
import numpy as np
from scipy.interpolate import splrep, splev
from pymavlink import mavutil
from pymavlink import mavwp
from bisect import bisect_left
wp = mavwp.MAVWPLoader()

master = mavutil.mavlink_connection('udpin:localhost:14551') # Mavlink
master.wait_heartbeat()
print("mavlink connected")

def request_message_interval(message_id: int, frequency_hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        1e6 / frequency_hz,  # The interval between two messages in microseconds.
        # set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0,  # Unused parameters
        0,  # Target address of message stream (if message has target address fields).
        # 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_REACHED ,10)


def request_mission_list(timeout_seconds=10):
    start_time = time.time()  # Record the starting time

    while True:
        master.mav.send(mavutil.mavlink.MAVLink_mission_request_list_message(
            master.target_system, master.target_component, 0))
        
        msg = master.recv_match(type=['MISSION_COUNT'], blocking=False)
        time.sleep(0.1)
        if msg:
            print(msg)
            return msg  # Return the received message
        
        if time.time() - start_time > timeout_seconds:  # Check if elapsed time has exceeded the timeout
            print("Request timed out.")
            return None  # Return None to indicate a failure due to timeout
        else:
            print('Trying to request mission list')


def read_wp():
    wp_list=[]
    msg = request_mission_list()

    for i in range(msg.count):
        master.waypoint_request_send(i)
        msg = master.recv_match(type=['MISSION_ITEM'],blocking=True)
        wp_list.append([(msg.seq),
                        (msg.frame),
                        (msg.command),
                        (msg.current),
                        (msg.autocontinue),
                        (msg.param1),(msg.param2),(msg.param3),(msg.param4),
                        (msg.x),(msg.y),(msg.z),
                        (msg.mission_type)])
    return wp_list


def clear_wp(timeout_seconds=10):
    start_time = time.time()  # Record the starting time

    while True:
        master.mav.send(mavutil.mavlink.MAVLink_mission_clear_all_message(
            master.target_system, master.target_component, 0))

        msg = master.recv_match(type=['MISSION_ACK'], blocking=False)
        time.sleep(0.1)
        if msg:
            print(msg)
            return msg  # Optionally, return the received message if needed

        if time.time() - start_time > timeout_seconds:  # Check if elapsed time has exceeded the timeout
            print("Clearing waypoints timed out.")
            return None  # Return None to indicate a failure due to timeout
        
        print('Trying to clear wp list')


def update_mission(wp_list,new_point):
    done = False
    while not done:
        if new_point:
            # print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- NEW POINT =-=-=-=-=-=-=-=-=-=-=-=--=--=-=")
            position = bisect_left([x[0] for x in wp_list], new_point[0])
            if 17 <= wp_list[position][2] <= 19:
                wp_list[position] = new_point
            else:
                wp_list.insert(position, new_point)
                for i in range(position+1, len(wp_list)):
                    wp_list[i][0] += 1
        
        wp.clear()

        for mission_item in range(len(wp_list)): 

            seq = wp_list[mission_item][0]
            frame = wp_list[mission_item][1]
            command = wp_list[mission_item][2]
            current = wp_list[mission_item][3]
            autocontinue = wp_list[mission_item][4]
            param1,param2,param3,param4 = wp_list[mission_item][5],wp_list[mission_item][6],wp_list[mission_item][4],wp_list[mission_item][8]
            x,y,z = wp_list[mission_item][9],wp_list[mission_item][10],wp_list[mission_item][11]
            mission_type = wp_list[mission_item][12]

            
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component,
                                                            seq, frame, command,
                                                            current, autocontinue,
                                                            param1, param2, param3, param4,
                                                            x, y, z,mission_type)
            wp.add(p)

        print('NEW WAYPOINT COUNT : ',wp.count())
        a = clear_wp() 
        if a:
            pass
        else:
            continue
        master.waypoint_count_send(wp.count())

        for i in range(wp.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
            # print(msg)
            master.mav.send(wp.wp(msg.seq))
            # print(f'Sending waypoint {msg.seq}')
            done = True


def loiter_point(lat,lon,frame=3,command=17,current=0,autocontinue=1,param1=0,param2=0,radius=50,param4=1.0,z=50.0,mission_type=0):
    msg = master.recv_match(type=['MISSION_ITEM_REACHED'],blocking=True)
    seq = 1 + msg.seq
    # param1 = turns
    param3 = radius
    x = lat
    y = lon
    print('+++++++++',seq,'+++++++++')
    return [seq,frame,command,current,autocontinue,param1,param2,param3,param4,x,y,z,mission_type]



def MAV_CMD_NAV_WAYPOINT(WP:int):
    master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
        WP,  # The command index/sequence number of the command to jump to.
        0,  # Number of times that the DO_JUMP command will execute before moving to the next sequential command. If the value is zero the next command will execute immediately. A value of -1 will cause the command to repeat indefinitely. 
        0,         0,         0,        0,        0)    
    

def flight_mode(mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def loiter(lat,lon):
    otime = time.time()
    wp_list = read_wp()
    new_point = loiter_point(lat,lon)  
    update_mission(wp_list,new_point)
    flight_mode('AUTO')
    MAV_CMD_NAV_WAYPOINT(new_point[0])
    ntime = time.time()
    print(ntime-otime)

def carry_on():
    msg = master.recv_match(type=['MISSION_ITEM_REACHED'],blocking=True)
    seq = 2 + msg.seq
    flight_mode('AUTO')
    MAV_CMD_NAV_WAYPOINT(seq)


mision_list = [[0, 0, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.36323928833008, 149.16522216796875, 584.0, 0]
               , [1, 3, 22, 0, 1, 15.0, 0.0, 0.0, 0.0, -35.36326217651367, 149.1652374267578, 50.0, 0]
               , [2, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.358699798583984, 149.16436767578125, 50.0, 0]
               , [3, 3, 18, 0, 1, 200.0, 0.0, 30.0, 0.0, -35.35601043701172, 149.159423828125, 50.0, 0]
               , [4, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.359615325927734, 149.15618896484375, 50.0, 0]
               , [5, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.367088317871094, 149.1580810546875, 50.0, 0]
               , [6, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.36829376220703, 149.16258239746094, 50.0, 0]
               , [7, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.36640548706055, 149.16542053222656, 30.0, 0]
               , [8, 0, 178, 0, 1, 1.0, 20.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0]
               , [9, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0, -35.364967346191406, 149.1654815673828, 15.0, 0]
               , [10, 3, 21, 0, 1, 0.0, 0.0, 0.0, 1.0, -35.363277435302734, 149.1652374267578, 0.0, 0]
               , [11, 0, 20, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]]


def lock(lat , lon , elev):

    mav = master.recv_match(type='AHRS2', blocking=True)
    phi = mav.roll
    theta = mav.pitch
    psi = mav.yaw
    altitude = mav.altitude
    latitude = mav.lat * 1e-7
    longtiude = mav.lng * 1e-7

    lat1, lon1, elev1 = latitude, longtiude, altitude  # (dynamic)

    lat1_rad, lon1_rad = np.deg2rad(lat1), np.deg2rad(lon1)
    lat2_rad, lon2_rad = np.deg2rad(lat), np.deg2rad(lon)

    R = 6371 # Earth radius in kilometers
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine
    a = np.sin(dlat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c


    bearing = np.arctan2(np.sin(dlon) * np.cos(lat2_rad),
                        np.cos(lat1_rad) * np.sin(lat2_rad) -
                        np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon))

    x_distance = 1000 * distance * np.cos(bearing)
    y_distance = 1000 * distance * np.sin(bearing)
    z_distance = elev1 - elev

    v_ned = np.array([x_distance, y_distance, z_distance]) 

    R_bn = np.array([
    [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
    [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
    [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]
    ])

    v = np.matmul(R_bn, v_ned)

    Azimuth = np.arctan2(v[1], v[0]) 
    Polar = np.arctan2(v[2], np.sqrt(v[0]**2 + v[1]**2)) 

    Azimuth = np.rad2deg(Azimuth)
    Polar = np.rad2deg(Polar)

    return Azimuth , Polar , distance*1000 , np.rad2deg(bearing), psi, lat1,lon1,elev1


def designate(azimuth,elevation, perc: bool = False):
    mav = master.recv_match(type='AHRS2', blocking=True)
    phi = mav.roll
    theta = mav.pitch
    psi = mav.yaw
    lat1 = mav.lat * 1e-7
    lon1 = mav.lng * 1e-7
    altitude = mav.altitude
    azimuth = np.deg2rad(azimuth)
    elevation = np.deg2rad(elevation)

    # Convert input values from degrees to radians
    lat1,lon1,elev1 = altitude, azimuth, altitude

    elevation = np.deg2rad(elevation)
    lat1_rad = np.deg2rad(lat1)
    lon1_rad = np.deg2rad(lon1)

    # Earth radius in meters
    radius = 6371000

    # Calculate the components of unit vector based on the input angles 
    x = np.cos(azimuth) * np.cos(elevation)
    y = np.sin(azimuth) * np.cos(elevation)
    z = np.sin(elevation)
    # Create the unit vector
    unit_vector = np.array([x, y, z])

    # Transform the unit_vector from the body frame to the NED frame
    R_bn = np.array([
        [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
        [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]
        ])
    v_ned = np.matmul(R_bn.T, unit_vector)

    # Calculate the gimbal angles in NED frame
    bearing = np.arctan2(v_ned[1], v_ned[0])  # Bearing angle in radians
    n_elevation = np.arctan2(v_ned[2], np.sqrt(v_ned[0]**2 + v_ned[1]**2))  # Elevation angle, downwards

    report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
    home_alt = report.terrain_height

    # estimate the target elevation to be the same as home elevation
    distance = (altitude - home_alt) / np.tan(n_elevation)

    # Calculate the coordinates of the estimated point using the haversine formula
    angular_distance = distance / radius
    lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
                        np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
                                    np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # Convert the latitude and longitude from radians to degrees
    lat = np.degrees(lat_rad)
    lon = np.degrees(lon_rad)

    # check terrain at estimated point
    master.mav.send(mavutil.mavlink.MAVLink_terrain_check_message(int(lat*1e7), int(lon*1e7)))
    report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
    elev = report.terrain_height

    # calculate the new distance based on terrain data
    distance = (altitude - elev) / np.tan(n_elevation)
    # Calculate the coordinates of the estimated point using the haversine formula
    angular_distance = distance / radius
    lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
                        np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
                                    np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # Convert the latitude and longitude from radians to degrees
    lat = np.degrees(lat_rad)
    lon = np.degrees(lon_rad)

    return lat, lon, elev, distance, np.rad2deg(bearing), psi, lat1,lon1,elev1


def decimal_degrees_to_dms(Decimal,lat:bool = False, lon:bool = False ):
    d = int(Decimal)
    m = int((Decimal - d) * 60)
    s = round((Decimal - d - m/60) * 3600.00,2)

    if lat == True:
        if d < 0:
            H = 'S'
        else:
            H = 'N'

    if lon == True:
        if d < 0:
            H = 'W'
        else:
            H = 'E'
    
    return (f"{abs(d)}°{abs(m)}'{abs(s)}"f'"{H}')


def ranging(azimuth,elevation, plot=False):
    o_time = time.time()
    mav = master.recv_match(type='AHRS2', blocking=True)
    phi = mav.roll
    theta = mav.pitch
    psi = mav.yaw
    latitude = mav.lat * 1e-7
    longtiude = mav.lng * 1e-7
    altitude = mav.altitude
    azimuth = np.deg2rad(azimuth)
    elevation = np.deg2rad(elevation)

    # Convert input values from degrees to radians
    lat1, lon1, elev1 = latitude, longtiude, altitude  # (dynamic)

    lat1_rad = np.deg2rad(lat1)
    lon1_rad = np.deg2rad(lon1)

    # Earth radius in meters
    radius = 6371000

    # Calculate the components of unit vector based on the input angles 
    x = np.cos(azimuth) * np.cos(elevation)
    y = np.sin(azimuth) * np.cos(elevation)
    z = np.sin(elevation)

    # Create the unit vector
    unit_vector = np.array([x, y, z])

    # Transform the unit_vector from the body frame to the NED frame
    R_bn = np.array([
        [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
        [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]
        ])
    v_ned = np.matmul(R_bn.T, unit_vector)

    # Calculate the gimbal angles in NED frame
    bearing = np.arctan2(v_ned[1], v_ned[0])  # Bearing angle in radians
    n_elevation = np.arctan2(v_ned[2], np.sqrt(v_ned[0]**2 + v_ned[1]**2))  # Elevation angle, downwards

    report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
    home_alt = report.terrain_height

    # Calculate the coordinates of the estimated point using the haversine formula
    distance = (altitude - home_alt) / np.tan(n_elevation) # estimate the target elevation to be the same as home elevation
    angular_distance = distance / radius
    lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
                        np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
                                    np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # Convert the latitude and longitude from radians to degrees
    lat = np.degrees(lat_rad)
    lon = np.degrees(lon_rad)

    # check terrain at estimated point
    master.mav.send(mavutil.mavlink.MAVLink_terrain_check_message(int(lat*1e7), int(lon*1e7)))
    report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
    elev = report.terrain_height
    point1 = (distance,elev)
    point1_distance = distance
    
    # draw terrain in the line of sight
    x = []
    y = []

    for i in range(int(point1_distance)-200,int(point1_distance)+350,2):
        distance = i
        # distance = (altitude - home_alt) / np.tan(n_elevation)
        angular_distance = distance / radius
        lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
                            np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
        lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
                                        np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
        
        # Convert the latitude and longitude from radians to degrees
        lat = np.degrees(lat_rad)
        lon = np.degrees(lon_rad)

        # check terrain at estimated point
        master.mav.send(mavutil.mavlink.MAVLink_terrain_check_message(int(lat*1e7), int(lon*1e7)))
        report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
        elev = report.terrain_height
        x.append(distance)
        y.append(elev)

    deviation = np.diff(y)
    threshold = np.std(deviation) * 0.2  # 0.2 of the standard deviation

    spikes = list(np.where(np.abs(deviation) > threshold)[0] + 1)

    for spike in spikes:
        if spike != 0 and spike != len(y) - 1: # Avoid spikes at the boundaries
            y[spike] = (y[spike - 1] + y[spike + 1]) / 2

    # Fit a cubic spline (k=3) to the sensor data
    spline_params = splrep(x, y, k=3)

    # Create a fine grid of x-values for evaluation
    grid = 10000
    x = np.linspace(x[0], x[-1], grid)

    # Evaluate the spline at the fine grid of x-values
    # graph_y = y  # 3d plot
    y = splev(x, spline_params)

    slope = np.tan(-n_elevation)
    x1 = np.linspace(x[0],x[len(x)-1],len(x))
    y1 = altitude + slope * x1

    # Find the difference between the sensor data and line values
    differences = np.abs(y - y1)

    # Find the index of the first intersection point
    index_of_intersection = np.argmin(differences)

    # Extract the intersection point
    intersection_point = (x[index_of_intersection], y[index_of_intersection])


    distance = intersection_point[0]
    # distance = (altitude - home_alt) / np.tan(n_elevation)
    angular_distance = distance / radius
    lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
                        np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
                                    np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # Convert the latitude and longitude from radians to degrees
    lat = np.degrees(lat_rad)
    lon = np.degrees(lon_rad)
    elev = intersection_point[1]
    print(intersection_point)
    n_time = time.time()
    total = n_time - o_time
    print(total)
    
    # if plot:
    #     '''Terrain plot'''
    #     plt.plot(x1, y1, label='Line of sight')
    #     plt.plot(x, y, '-.', label='Terrain')

    #     plt.xlabel('Distance (m)')
    #     plt.ylabel('Altitude (m)')

    #     plt.grid(True)
    #     plt.scatter(*point1, color='red', marker= 'x', label='Esitmated point')
    #     # plt.scatter(*point2, color='blue', label='corrected point')
    #     plt.scatter(*intersection_point, color='green', label='First Intersection Point')
    #     plt.axhline(y=home_alt, color='black', linestyle='--', label='Home altitude')

    #     plt.axis('equal')  # To maintain aspect ratio
    #     plt.legend()
    #     plt.show()

    """3D A-&-G Plot"""
    # Apoint = (lat1, lon1, altitude)
    # Gpoint = (lat, lon, elev)


    # distance = x[-1]
    # # distance = (altitude - home_alt) / np.tan(n_elevation)
    # angular_distance = distance / radius
    # lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
    #                     np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    # lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
    #                                 np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # g = np.degrees(lat_rad)
    # h = np.degrees(lon_rad)

    # distance = x[0]
    # # distance = (altitude - home_alt) / np.tan(n_elevation)
    # angular_distance = distance / radius
    # lat_rad = np.arcsin(np.sin(lat1_rad) * np.cos(angular_distance) +
    #                     np.cos(lat1_rad) * np.sin(angular_distance) * np.cos(bearing))
    # lon_rad = lon1_rad + np.arctan2(np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1_rad),
    #                                 np.cos(angular_distance) - np.sin(lat1_rad) * np.sin(lat_rad))
    
    # j = np.degrees(lat_rad)
    # k = np.degrees(lon_rad)

    # # Interpolate between the two points to create the axis
    # grid = len(graph_y)
    # axis_x = np.linspace(j, g, grid)
    # axis_y = np.linspace(k, h, grid)
    # print(type(graph_y))
    # axis_z = graph_y



    # # Plot the two points
    # ax.scatter(*Apoint, c='cyan', label='Air')
    # ax.scatter(*Gpoint, c='brown', label='Ground')

    # # Label the axes
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # # Plot the sensor data along the axis
    # ax.scatter(axis_x, axis_y, axis_z, c='purple', s=2, label='Terrain')

    # # Add a legend
    # # ax.legend()

    # # Show the plot
    
    # plt.draw()
    # plt.pause(1)

    return lat, lon, elev, distance, np.rad2deg(bearing), psi, lat1,lon1,elev1
