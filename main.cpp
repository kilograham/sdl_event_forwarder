/**
 * Copyright (c) 2021 Graham Sanderson
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "SDL.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int set_interface_attribs(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t) speed);
    cfsetispeed(&tty, (speed_t) speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void send_event(int fd, const uint8_t *buf, uint buf_len) {
    uint len = write(fd, buf, buf_len);
    if (len != buf_len) {
        printf("Failed to write!\n");
    }
}

int controller_index = -1;
SDL_GameController *controller;

void update_controller(int which) {
    if (which == -1) {
        for (int i = 0; i < SDL_NumJoysticks(); i++) {
            if (SDL_IsGameController(i)) {
                which = i;
                break;
            }
        }
    }
    if (which != -1) {
        controller_index = which;
        controller = SDL_GameControllerOpen(which);
        printf("Opening controller %d\n", which);
    }
}

int main(int argc, char *argv[]) {
    SDL_Window *window;

    if (argc != 2) {
        printf("Usage: sdl_event_forwarder <ttydev>\n");
        return -1;
    }

    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", argv[1], strerror(errno));
        return -1;
    }
    set_interface_attribs(fd, B115200);

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);

    window = SDL_CreateWindow(
            "Pico SDL Event Forwarder",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            640,
            480,
            SDL_WINDOW_OPENGL
    );

    if (!window) {
        printf("Could not create window: %s\n", SDL_GetError());
        return 1;
    }

    uint8_t buf[16];
    bool done = false;
    SDL_Event event;
    update_controller(-1);
    while (!done && SDL_WaitEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                // printf("%d\n", event.key.keysym.scancode);
                buf[0] = 26;
                buf[1] = 0;
                buf[2] = event.key.keysym.scancode;
                send_event(fd, buf, 3);
                break;
            case SDL_KEYUP:
                buf[0] = 26;
                buf[1] = 1;
                buf[2] = event.key.keysym.scancode;
                send_event(fd, buf, 3);
                break;
            case SDL_MOUSEBUTTONDOWN:
                buf[0] = 26;
                buf[1] = 2;
                buf[2] = event.button.button;
                send_event(fd, buf, 3);
                break;
            case SDL_MOUSEBUTTONUP:
                buf[0] = 26;
                buf[1] = 3;
                buf[2] = event.button.button;
                send_event(fd, buf, 3);
                break;
            case SDL_MOUSEMOTION:
                buf[0] = 26;
                buf[1] = 4;
                buf[2] = std::max(-128, std::min(event.motion.xrel, 127));
                buf[3] = std::max(-128, std::min(event.motion.yrel, 127));
                send_event(fd, buf, 4);
                break;
            case SDL_CONTROLLERDEVICEADDED:
                if (controller_index != -1) {
                    printf("closing controller %d\n", controller_index);
                    SDL_GameControllerClose(controller);
                }
                update_controller(event.cdevice.which);
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                if (event.cdevice.which == controller_index) {
                    update_controller(-1);
                }
                break;
            case SDL_CONTROLLERBUTTONDOWN:
            case SDL_CONTROLLERBUTTONUP:
                if (controller) {
                    uint8_t state = 0;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP)) state |= 0x10;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) state |= 0x20;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) state |= 0x40;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) state |= 0x80;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X) ||
                        SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER))
                        state |= 0x1;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y)) state |= 0x2;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A) ||
                        SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER))
                        state |= 0x4;
                    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B)) state |= 0x8;
                    buf[0] = 26;
                    buf[1] = 5;
                    buf[2] = state;
                    send_event(fd, buf, 3);
                }
                break;
            case SDL_QUIT:
                done = true;
                break;
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}