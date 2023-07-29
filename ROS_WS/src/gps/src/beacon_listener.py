import subprocess
import time
import urllib.request

ssid_name = "Isaac_Asimov"

def what_wifi():
    process = subprocess.run(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip().split(':')[1]
    else:
        return ''

def is_connected_to(ssid: str):
    return what_wifi() == ssid

def scan_wifi():
    process = subprocess.run(['nmcli', '-t', '-f', 'SSID,SECURITY,SIGNAL', 'dev', 'wifi'], stdout=subprocess.PIPE)
    if process.returncode == 0:
        return process.stdout.decode('utf-8').strip().split('\n')
    else:
        return []
        
def is_wifi_available(ssid: str):
    return ssid in [x.split(':')[0] for x in scan_wifi()]

def connect_to(ssid: str):
    if not is_wifi_available(ssid):
        return False
    subprocess.call(['nmcli', 'dev', 'wifi', 'connect', ssid])
    return is_connected_to(ssid)


if __name__ == "__main__":
    print("Attempting to connect to ", ssid_name)
    while not connect_to(ssid_name):
        time.sleep(1.0)

    print("Success!")

    

    

