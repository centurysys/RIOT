#ifndef _UART1_H
#define _UART1_H

/* for TX request */
typedef struct msgbuf {
    char code;
    char *buf;
    int len;
} msgbuf_t;

enum uart1_msg_type_t {
    MSG_UART_RECEIVED,

    MSG_RF_RECEIVED,
    MSG_SEND_REQ,
};


extern kernel_pid_t uart1_handler_pid;

void uart1_thread_init(void);

#endif
