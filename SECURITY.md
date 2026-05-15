# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability in this project, please **do not open a public GitHub issue**.

Instead, report it privately by emailing:

**gieselman.kyle@gmail.com**

Please include:
- A description of the vulnerability and its potential impact
- Steps to reproduce or a proof-of-concept
- Any suggested mitigations, if you have them

I aim to acknowledge reports within **48 hours** and will work with you to understand and address the issue promptly.

## Scope

This is an embedded firmware project for an STM32H563-based integrated motor controller. Security considerations include:

- Memory safety (buffer overflows, stack corruption)
- Communication interface integrity (UART/USB/SPI/I2C)
- Flash programming and bootloader integrity
- Debug interface exposure (SWD/JTAG)

## Out of Scope

- Theoretical vulnerabilities with no practical exploit path on embedded hardware
- Physical attacks requiring direct hardware access beyond normal debugging interfaces
