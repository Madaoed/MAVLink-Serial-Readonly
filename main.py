from pymavlink import mavutil
import serial
import time

if __name__ == '__main__':
    #mav_connection = mavutil.mavlink_connection(f'{PROTOCOL}:{HOST}:{PORT}')
    mav_connection = mavutil.mavlink_connection('COM3', baud=57600)
    mav_connection.wait_heartbeat()

    while True:
        localtime = time.asctime( time.localtime(time.time()) )
        print(localtime)
        #msg =str(mav_connection.recv_match(type='GPS_RAW_INT',blocking=True))
        msg = str(mav_connection.recv_match(type='GPS_RAW_INT', blocking=True))
        print(msg)
        #msgSplit = msg.split(', ')
        #gpstime=
        # lat=msgSplit[2].replace('lat : ', '')
        # lat = float(lat)*0.0000001
        # lon=msgSplit[3].replace('lon : ', '')
        # lon = float(lon)*0.0000001
        # alt=msgSplit[4].replace('alt : ', '')
        # alt = float(alt)*0.0001
        # print(lat)
        # print(lon)
        # print(alt)


