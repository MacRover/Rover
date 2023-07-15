#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/**
 * @brief Send CAN frame over the given socket
 * @param target_socket file descriptor pointing to CAN socket
 * @param frame data to transmit
 * @return
 */
int send_can(const int target_socket, const can_frame frame)
{
    if (write(target_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return 1;
    }
    return 0;
}

/**
 * @brief Create and bind CAN socket
 * @param target_socket target file descriptor for CAN socket
 * @param device CAN device name as a string
 * @return
 */
int init_can(int &target_socket, const char *device)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((target_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, device);
    ioctl(target_socket, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(target_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }
    return 0;
}

/**
 * @brief Safely close CAN socket
 * @param target_socket file descriptor pointing to CAN socket
 * @return
 */
int close_can(int &target_socket)
{
    if (close(target_socket) < 0)
    {
        perror("Close");
        return 1;
    }
    return 0;
}

/**
 * @brief Construct a CAN frame
 * @param can_id frame ID
 * @param dlc number of bytes to send
 * @param data data to send
 * @param target_frame pointer to can_frame struct
 * @return
 */
int build_can_frame(const canid_t can_id, const u_int8_t dlc, const u_int8_t *data, can_frame &target_frame)
{

    if (dlc > CAN_MAX_DLEN)
    {
        // ensure dlc is in acceptable range
        return 1;
    }
    target_frame.can_id = can_id;
    target_frame.can_dlc = dlc;
    memcpy(target_frame.data, data, dlc);
    return 0;
}

int main(int argc, char **argv)
{
    int s;
    struct can_frame frame;
    const char device[] = "vcan0";

    init_can(s, device);

    u_int8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    build_can_frame(0x555, 8, data, frame);

    send_can(s, frame);

    close_can(s);

    return 0;
}
