#!/usr/bin/env python3
import wifi
import sys

ssid_name = "Isaac_Asimov"
ip_addr = "10.10.11.1"

if __name__ == "__main__":
    led_mode = sys.argv[1] if len(sys.argv) > 1 else ""

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
