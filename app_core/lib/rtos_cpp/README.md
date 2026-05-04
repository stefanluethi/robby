
## Development

### Prerequisites

Todo: Create Docker container

- arm-none-eabi-gcc
- arm-none-eabi-newlib
- qemu-system-arm

### Run

```shell
 qemu-system-arm -machine lm3s6965evb -nographic -no-reboot -kernel cmake-build-debug/test/integration/rtos_integration_test
```

```shell
qemu-system-arm \
    -cpu cortex-m3 \
    -machine lm3s6965evb \
    -nographic \
    -no-reboot \
    -gdb tcp::3333 \
    -S \
    -kernel cmake-build-debug/test/integration/rtos_integration_test
```
