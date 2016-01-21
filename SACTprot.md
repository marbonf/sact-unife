# SACT interface protocol #

The SACT protocol provides a simple master/slave interface with the SACT board (playing the slave role). Currently, it supports
two different modes, an ASCII mode and a binary mode, and two communication channels, namely the RS-232 and the Ethernet connection
with an XPort / MatchPort Serial-to-Ethernet device server.

Initially, the board is in a NO SYNC (unconnected) mode and waits for a sequence of ASCII commands that can be either:

```
"SYNC0" + CR/LF
"SYNC1" + CR/LF
"SYNCA" + CR/LF
```

that enables the ASCII mode of the SACT protocol, or

```
"SYNC0" + CR/LF
"SYNC1" + CR/LF
"SYNCB" + CR/LF
```

that enables the binary mode. The command sequence is monitored on both communication channels, but if it is received on one of the
two, the other is disabled until disconnection on the activated one.

In ASCII mode, human-readable strings are exchanged between master and slave. The full list of commands and messages can be found
in the source file `SACT_Protocol.c`.

In binary mode, instead, the interaction is based on byte packets of four types:
  * SACT Command Packet (SCP, from master to slave)
  * SACT Sensor Packet (SSP, from slave to master)
  * SACT Diagnostic Packet (SDP, from slave to master)
  * SACT Parameter Packet (SPP, from slave to master)

## SACT Command Packet ##

| FIELD | Bytes | VALUE | Comment |
|:------|:------|:------|:--------|
| Header | 2     | 0xAA 0x55 |  Header |
| Byte count | 1     | N     | N of bytes in the packet (escluding header and byte count itself) |
| Command | 1     | 0-255 | See `SACT_Protocol.c` for the complete list |
| Data  | 2xArgsN | xx    | Data, depending on the number of arguments required by the command |
| CRC   | 2     | xx    | CRC 16, calculated on all previous bytes excluding header and byte count. |
| EOP   | 1     | 0xFF  | End of Packet |


## SACT Sensor Packet ##

| FIELD | Bytes | VALUE | Comment |
|:------|:------|:------|:--------|
| Header | 2     | 0xAA 0x55 |  Header |
| Byte count | 1     | N     | N of bytes in the packet (escluding header and byte count itself) |
| Packet type | 1     | 0xA1  | Identifies SSP |
| Config flags | 2     | 0-0xFFFF | Packet configuration bits (see `SACT_Protocol.h` for bits spec.) |
| Data  | n     | xx    | Sensor Data |
| CRC   | 2     | xx    | CRC 16, calculated on all previous bytes excluding header and byte count. |
| EOP   | 1     | 0xFF  | End of Packet |

## SACT Diagnostic Packet ##

| FIELD | Bytes | VALUE | Comment |
|:------|:------|:------|:--------|
| Header | 2     | 0xAA 0x55 |  Header |
| Byte count | 1     | N     | N of bytes in the packet (escluding header and byte count itself) |
| Packet type | 1     | 0xB1  | Identifies SDP |
| Status flags | 1     | 0-255 | Status flags (e.g. overcurrent, comm. error etc.) |
| Control Mode | 1     | 0-5   | Current SACT control mode |
| Last command | 1     | 0-255 | Last command received |
| Firmware version | 2     |       | First byte: major revision, second byte: minor revision |
| Board revision | 1     | 0-255 | Board revision number |
| CRC   | 2     | xx    | CRC 16, calculated on all previous bytes excluding header and byte count. |
| EOP   | 1     | 0xFF  | End of Packet |

## SACT Parameter Packet ##

| FIELD | Bytes | VALUE | Comment |
|:------|:------|:------|:--------|
| Header | 2     | 0xAA 0x55 |  Header |
| Byte count | 1     | N     | N of bytes in the packet (escluding header and byte count itself) |
| Packet type | 1     | 0xC1  | Identifies SPP |
| Parameter number | 1     | 0-255 | see `SACT_Protocol.c` for the complete list |
| Parameter value  | 2     | 0-0xFFFF | Current parameter value |
| CRC   | 2     | xx    | CRC 16, calculated on all previous bytes excluding header and byte count. |
| EOP   | 1     | 0xFF  | End of Packet |