common_test_src = files(
    'test_debug.cpp',
    'test_ringbuf.cpp',
)

stm32f303_test_src = common_test_src + files(
    'main.cpp',
    'startup.cpp',
    'syscalls.cpp',
)

native_test_src = common_test_src + files(
    'main_native.cpp',
)
