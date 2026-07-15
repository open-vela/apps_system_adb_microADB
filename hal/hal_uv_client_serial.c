/*
 * Copyright (C) 2026 Xiaomi Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "adb.h"
#include "hal_uv_priv.h"
#include <uv.h>

/****************************************************************************
 * Private types
 ****************************************************************************/

typedef struct adb_client_serial_s {
    adb_client_uv_t uc;
    /* libuv handle must be right after adb_client_uv_t */
    uv_tty_t tty;
} adb_client_serial_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void serial_uv_allocate_frame(uv_handle_t* handle,
                       size_t suggested_size, uv_buf_t* buf) {
    UNUSED(suggested_size);

    adb_client_serial_t *client =
        container_of(handle, adb_client_serial_t, tty);
    adb_uv_allocate_frame(&client->uc, buf);
}

static void serial_uv_on_data_available(uv_stream_t* handle,
        ssize_t nread, const uv_buf_t* buf) {
    adb_client_serial_t *client =
        container_of(handle, adb_client_serial_t, tty);

    adb_uv_on_data_available(&client->uc, handle, nread, buf);
}

static int serial_uv_write(adb_client_t *c, apacket *p) {
    int ret;
    uv_buf_t buf;
    apacket_uv_t *up = container_of(p, apacket_uv_t, p);
    adb_client_serial_t *client =
        container_of(c, adb_client_serial_t, uc.client);

    buf = uv_buf_init((char*)&p->msg,
        sizeof(p->msg) + p->msg.data_length);

    up->wr.data = &client->uc;

    ret = uv_write(&up->wr, (uv_stream_t*)&client->tty, &buf, 1,
        adb_uv_after_write);
    if (ret) {
        adb_err("uv_write failed %d %d\n", ret, errno);
        return ret;
    }

    return 0;
}

static void serial_uv_kick(adb_client_t *c) {
    adb_client_serial_t *client =
        container_of(c, adb_client_serial_t, uc.client);

    if (!uv_is_active((uv_handle_t*)&client->tty)) {
        int ret = uv_read_start((uv_stream_t*)&client->tty,
            serial_uv_allocate_frame,
            serial_uv_on_data_available);
        assert(ret == 0);
    }

    adb_client_kick_services(c);
}

static void serial_uv_on_close(uv_handle_t* handle) {
    adb_client_serial_t *client =
        container_of(handle, adb_client_serial_t, tty);

    adb_uv_close_client(&client->uc);
}

static void serial_uv_close(adb_client_t *c) {
    adb_client_serial_t *client =
        container_of(c, adb_client_serial_t, uc.client);

    uv_close((uv_handle_t*)&client->tty, serial_uv_on_close);
}

static const adb_client_ops_t adb_serial_uv_ops = {
    .write = serial_uv_write,
    .kick  = serial_uv_kick,
    .close = serial_uv_close
};

static int serial_uv_config(uv_tty_t *tty, unsigned int baudrate) {
    struct termios tio;
    speed_t speed;
    uv_os_fd_t fd;
    int ret;

    ret = uv_fileno((uv_handle_t*)tty, &fd);
    if (ret) {
        adb_err("uv_fileno failed %d\n", ret);
        return ret;
    }

    if (tcgetattr(fd, &tio) < 0) {
        adb_err("tcgetattr failed %d\n", errno);
        return -errno;
    }

    switch (baudrate) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        case 1500000: speed = B1500000; break;
        case 3000000: speed = B3000000; break;
        default:
            adb_err("unsupported baudrate %u\n", baudrate);
            return -EINVAL;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        adb_err("tcsetattr failed %d\n", errno);
        return -errno;
    }

    ret = uv_tty_set_mode(tty, UV_TTY_MODE_RAW);
    if (ret) {
        adb_err("uv_tty_set_mode failed %d\n", ret);
        return ret;
    }

    return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int adb_uv_serial_setup(adb_context_uv_t *adbd, const char *path,
                        unsigned int baudrate) {
    adb_client_serial_t *client;
    int ret;
    int fd;

    client = (adb_client_serial_t*)adb_uv_create_client(sizeof(*client));
    if (client == NULL) {
        adb_err("failed to allocate serial client\n");
        return -ENOMEM;
    }

    client->uc.client.ops = &adb_serial_uv_ops;

    fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        adb_err("failed to open serial device %s: %d\n", path, errno);
        adb_uv_close_client(&client->uc);
        return -errno;
    }

    ret = uv_tty_init(adbd->loop, &client->tty, fd, 0);
    if (ret) {
        adb_err("serial tty init error %d\n", ret);
        close(fd);
        adb_uv_close_client(&client->uc);
        return ret;
    }

    ret = serial_uv_config(&client->tty, baudrate);
    if (ret < 0) {
        uv_close((uv_handle_t*)&client->tty, serial_uv_on_close);
        return ret;
    }

    ret = uv_read_start((uv_stream_t*)&client->tty,
        serial_uv_allocate_frame,
        serial_uv_on_data_available);
    if (ret) {
        adb_err("serial read_start error %d\n", ret);
        uv_close((uv_handle_t*)&client->tty, serial_uv_on_close);
        return ret;
    }

    adb_log("serial transport started on %s @ %u baud\n", path, baudrate);
    return 0;
}
