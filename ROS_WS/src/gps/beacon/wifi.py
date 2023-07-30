#!/usr/bin/env python3
import subprocess
import urllib.request

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


def get_website_html(ip, mode=""):
    """
    Fetches website data from ip address

    Args: ip address and mode for LED (on or off)
    """
    res = ""
    full_url = "http://{}:80/{}".format(ip, mode)
    print(full_url)
    with urllib.request.urlopen(full_url) as con:
        res = con.read()
        con.close()
    
    return res
        