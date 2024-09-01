#ifndef _UART_H
#define _UART_H

int fputc(int c, FILE * stream);

int fputs(const char *restrict s, FILE *restrict stream);

int puts(const char *_ptr);

#endif