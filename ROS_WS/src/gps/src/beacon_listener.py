#!/usr/bin/env python3
import subprocess
import time
import urllib.request

ssid_name = "Isaac_Asimov"
ip_addr = "10.10.11.1"

def what_wifi():
    process = subprocess.run(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'device', 'wifi'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip().split(':')[1]
    else:
        return ''

def is_connected_to(ssid: str):
    return what_wifi() == ssid

def scan_wifi():
    process = subprocess.run(['nmcli', '-t', '-f', 'SSID,SECURITY,SIGNAL', 'device', 'wifi'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip().split('\n')
    else:
        return []
        
def is_wifi_available(ssid: str):
    return ssid in [x.split(':')[0] for x in scan_wifi()]

def connect_to(ssid: str):
    if not is_wifi_available(ssid):
        return False
    subprocess.call(['nmcli', 'device', 'wifi', 'connect', ssid])
    return is_connected_to(ssid)


def rescan_networks():
    subprocess.call(['nmcli', 'device', 'wifi', 'rescan'])


if __name__ == "__main__":
    rescan_networks()
    print("Attempting to connect to", ssid_name)
    connection = connect_to(ssid_name)
    if not connection:
        print("Connection failed")
        exit(0)

    print("Success!")

    with urllib.request.urlopen("http://{}:80".format(ip_addr)) as con:
        print(con.read())
        con.close()
