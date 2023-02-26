#include <sys/stat.h>
#include <cstdint>

#include "usart.hpp"
#include "usart_stm32f303.hpp"

using USART = usart::stm32f303::USART;

extern "C" {
int _write([[maybe_unused]] int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        while(USART::tx_free() == 0);
        USART::tx_byte(std::byte(*ptr++));
    }
    USART::tx_flush();
    return len;
}

extern int _end;

void *_sbrk(int incr) {
    static unsigned char *heap = nullptr;
    unsigned char *prev_heap;

    if (heap == nullptr) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

    heap += incr;

    return prev_heap;
}

int _close([[maybe_unused]] int file) {
    return -1;
}


int _fstat([[maybe_unused]] int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty([[maybe_unused]] int file) {
    return 1;
}

int _lseek([[maybe_unused]] int file, [[maybe_unused]] int ptr, [[maybe_unused]] int dir) {
    return 0;
}

void _exit([[maybe_unused]]int status) {
    __asm("BKPT #0");
}

void _kill([[maybe_unused]] int pid, [[maybe_unused]] int sig) {
    return;
}

int _getpid(void) {
    return -1;
}

int _read ([[maybe_unused]] int file, [[maybe_unused]] char * ptr, [[maybe_unused]] int len) {
    return -1;
}

int _gettimeofday([[maybe_unused]] struct timeval *tv, [[maybe_unused]] void *tzvp )
{
    return 0;
}

int _open([[maybe_unused]] const char *filename,
          [[maybe_unused]] int oflag[],
          [[maybe_unused]] int pmode)
{
    return 0;
}

}
