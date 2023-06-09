extern int errno;
extern int __io_putchar(int ch) __attribute__((weak));

int _write(int fd, char *buffer, unsigned int count)
{
	int DataIdx;
	for (unsigned int i = 0; i < count; i++) {
		__io_putchar(*buffer++);
	}
	return count;
}

int _read(int fd, void *buffer, unsigned int count)
{
    return -1;
}

int _close(int fd)
{
    return -1;
}

int _fstat(int fd, void *buffer)
{
    return -1;
}

int _isatty(int fd)
{
    return -1;
}

long _lseek(int fd, long offset, int origin)
{
    return -1;
}

int _getpid(void) {
	return -1;
}

int _kill(void) {
	return -1;
}

void _exit(int status)
{
}

