#include "pololu_i2c.h"
#include <stdio.h>

int main()
{
    pololu_i2c_adapter adapter;
    pololu_i2c_init(&adapter);

    // Replace with your actual serial port name
    const char *port_name = "/dev/ttyACM2";

    printf("Connecting to %s...\n", port_name);
    if(pololu_i2c_connect(&adapter, port_name) != 0)
    {
        fprintf(stderr, "Failed to connect to the adapter.\n");
        return 1;
    }
    printf("Connected.\n");

    // Get and print device info
    pololu_i2c_device_info info;
    if(pololu_i2c_get_device_info(&adapter, &info) == 0)
    {
        printf("Device Info:\n");
        printf("  Vendor ID: 0x%04X\n", info.vendor_id);
        printf("  Product ID: 0x%04X\n", info.product_id);
        printf("  Firmware Version: %s\n", info.firmware_version);
        printf("  Serial Number: %s\n", info.serial_number);
    }
    else
    {
        fprintf(stderr, "Failed to get device info.\n");
    }

    // Scan for I2C devices
    printf("\nScanning for I2C devices...\n");
    uint8_t found_addresses[128];
    int device_count = pololu_i2c_scan(&adapter, found_addresses, 128);

    if(device_count > 0)
    {
        printf("Found %d device(s):\n", device_count);
        for(int i = 0; i < device_count; ++i)
        {
            printf("  Address: 0x%02X\n", found_addresses[i]);
        }
    }
    else if(device_count == 0)
    {
        printf("No I2C devices found.\n");
    }
    else
    {
        fprintf(stderr, "An error occurred during the I2C scan: %s\n", pololu_i2c_error_string(device_count));
    }

    // Disconnect
    pololu_i2c_disconnect(&adapter);
    printf("\nDisconnected.\n");

    return 0;
}
