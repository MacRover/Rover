#!/usr/bin/env python3
import subprocess
import urllib.request
import sys

ssid_name = "Isaac_Asimov"
ip_addr = "10.10.11.1"

def what_wifi():
    process = subprocess.run(['iwgetid', '-r'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip()
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
    if not is_connected_to(ssid):
        subprocess.call(['nmcli', 'device', 'wifi', 'connect', ssid])
    else:
        print("Wifi already connected")
    return is_connected_to(ssid)


def rescan_networks():
    subprocess.call(['nmcli', 'device', 'wifi', 'rescan'])


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    
    rescan_networks()
    print("Attempting to connect to", ssid_name)
    connection = connect_to(ssid_name)
    if not connection:
        print("Connection failed")
        exit(0)

    print("Success!")
    print("http://{}:80/{}".format(ip_addr, arg))
    with urllib.request.urlopen("http://{}:80/{}".format(ip_addr, arg)) as con:
        print(con.read())
        con.close()
