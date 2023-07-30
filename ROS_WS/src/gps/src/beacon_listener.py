#!/usr/bin/env python3
import subprocess
import urllib.request
import sys

ssid_name = "Isaac_Asimov"
ip_addr = "10.10.11.1"

def what_wifi():
    """
    Returns the name of the currently connected network
    """
    process = subprocess.run(['iwgetid', '-r'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip()
    else:
        return ''

def is_connected_to(ssid):
    """
    Checks if a specific network is connected

    Args: name of ssid
    """
    return what_wifi() == ssid

def scan_wifi():
    """
    Scans for all nearby wifi networks

    Return: list containing ssid, security, and signal strength of every network
    """
    process = subprocess.run(['nmcli', '-t', '-f', 'SSID,SECURITY,SIGNAL', 'device', 'wifi'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip().split('\n')
    else:
        return []
        
def is_wifi_available(ssid):
    """
    Checks if a wifi network can be found from list of available networks

    Args: name of ssid
    """
    # format is SSID:SECURITY:SIGNAL
    return ssid in [x.split(':')[0] for x in scan_wifi()]

def connect_to(ssid):
    """
    Attempts to connect to a given network

    Args: name of ssid

    Returns: True/False for success/failure
    """
    
    # cannot connect if not available
    if not is_wifi_available(ssid):
        print("Unable to find", ssid)
        return False
    # connect only if its not already connected
    if not is_connected_to(ssid):
        subprocess.call(['nmcli', 'device', 'wifi', 'connect', ssid])
    else:
        print("Wifi already connected")
        return True
    return is_connected_to(ssid)


def rescan_networks():
    """
    Rescans for surrounding networks
    """
    subprocess.call(['nmcli', 'device', 'wifi', 'rescan'])


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    
    print("Scanning networks...")
    rescan_networks()
    print("Attempting to connect to", ssid_name)

    connection = connect_to(ssid_name)
    if not connection:
        print("Connection failed")
        exit(0)

    print("Success!")

    # fetch website and print
    full_url = "http://{}:80/{}".format(ip_addr, arg)
    print(full_url)
    with urllib.request.urlopen(full_url) as con:
        print(con.read())
        con.close()
