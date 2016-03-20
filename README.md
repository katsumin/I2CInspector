# I2CInspector

## What's this?
I2C monitoring firmware for LPC810. This firmware will snoop I2C bus to take logging of transactions through USART.

## Schematic
[![Schematic](https://raw.githubusercontent.com/toyoshim/I2CInspector/master/schem.png "Schematic")](https://upverter.com/toyoshim/8c36ac7ed77c5cf6/I2CInspector/)

## How to build
This repository includes prebiult binaries under Release/ directory, but aalso you can build it by yourself with the LPCXpresso project files included in this repository too.
You need to import CMSIS_CORE_LPX8xx and lpc800_driver_lib from <lpcxpresso>/Examples/Legacy/NXP/LPC800/LPC8xx_Libraries.zip to use these libraries.

## How to use
Connect I2C pins to any existing I2C but there master and slave coexist. Your device with this firmware does not behave as either master or slave. Just snoop the protocol and convert it to 230400Hz USART TX as text logs.

## Example log
```
I2CInspector ready
3E<0038
3E<0039
3E<0014
3E<007F
3E<0056
3E<006C
3E<0038
3E<000C
3E<0001
3E<0080
3E<4054
3E<4065
3E<4073
3E<4074
3E<4031
```

The first hex ``3E`` means the target address. Following ``<`` means that this is a write transaction. ``>`` can alternate this to represent a read transaction. Following hex is data. As I2C allows arbitorary length of data transaction, it can have different length from this example.

## Injecting I2C transaction
Also you can send I2C transactions as a master. But note that it does not care about multi-master systems. You yorself should be careful about sending transaction not being conflicted with other existing masters.

To write, you can just send the same format data through the USART RX. To read, you can send a read byte length after the ``>``. It would be something like ``3E>2`` to read 2 Bytes from a device having address 0x3E.

## Write to I2C device from PC through I2CInspector
You can send I2C write command from your PC through the I2CInspector connected via USB Serial. Here is a rough sketch for POSIX systems though it does not have something great for error checks.

```
#define USB_SERIAL "/dev/ttyUSB0"
const char kCommand[] = "3E<4032\n";
int fd = open(USB_SERIAL, O_RDWR);
...
struct termios termios;
tcgetattr(fd, &termios) || cfsetspeed(&termios, 230400) || tcsetattr(fd, TCSANOW, &termios);
write(fd, kCommand, sizeof(kCommand) - 1);  // Do not send the last NULL!
ssize_t done = 0;
char data[8];
while (done != (sizeof(kCommand) - 1))
  done += read(fd, data, 8);
```

