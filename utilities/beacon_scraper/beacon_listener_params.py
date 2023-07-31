#!/usr/bin/env python3
import wifi
import sys

if __name__ == "__main__":
    if len(sys.argv) <= 2:
        print("Enter name and ip of WiFi.")
        exit(0)
    
    (ssid_name, ip_addr) = sys.argv[1:3]
    led_mode = sys.argv[3] if len(sys.argv) > 3 else ""

    print("Scanning networks...")
    wifi.rescan_networks()
    print("Attempting to connect to", ssid_name)

    connection = wifi.connect_to(ssid_name)
    if not connection:
        print("Connection failed")
        exit(0)

    print("Success!")

    gps_data = wifi.get_website_html(ip_addr, led_mode)
    print(gps_data)

