# pacific-tpms
Experimental decoder and encoder for the Pacific PMV-107J TPMS (315MHz) sensors used by Toyota

I was inspired by the excellent TPMS work by Jared Boone (https://github.com/jboone/gr-tpms and https://github.com/jboone/tpms) that did have a packet_type listed that looked a lot like what I got out of my sensors, but no crc/checksum function was listed. Trying to make sense of the stated 64 bits got me some partial matches trying to bruteforce the CRC as described in the tpms project (using https://github.com/sitsec/bruteforce-crc), but it wasn't until I realized that the packets were actually 66 bits in length that it fit perfectly.

This is what I came up with after decoding a lot of messages from the real sensors (9 different ones) and sending custom messages to the TPMS unit in the car while watching the diagnostic tool:

FSK modulation at 315.98MHz, deviation around +/- 35kHz. Raw bitrate around 9910bps
Preamble 0001111110 (the last two bits are probably a first, ignored, manchester encoded bit) as the remaining message assumes a diffman decode starting at 1

Packet bit format after differential manchester decoding (MSB first)

| Offset | Length | Description            | Notes |
| ------ | ------ | ---------------------- | ------|
| 0      | 28     | Sensor ID              | 7 hex digits |
| 28     | 1      | Battery status         | 1 if low, 0 if ok or 'over' in Toyota speak |
| 29     | 2      | Counter                | Counts 1,2,3 for every message out of the sensor. Car seems to ignore this and happily accepts the same counter over and over without ever changing |
| 31     | 1      | Unknown                | Must be zero for the packet to be recognized by the car (haven't seen a real sensor ever set this bit) |
| 32     | 1      | Unknown                | Seems to be ignored by the car, sensors sets this to 0 |
| 33     | 1      | Self-test failed?      | My sensors sets this to 0 but when I send a message with this bit set to 1 to the car the TPMS light starts flashing and the values in the message are ignored |
| 34     | 8      | Tire pressure          | PSI/0.363 + 40 or kPa/2.48 + 40, diagnostics reports this in PSI |
| 42     | 8      | Inverted tire pressure | xor 0xff of above |
| 50     | 8      | Tire temperature       | Celsius + 40 resulting in a range from -40 to +215C, diagnostics insists on reporting this in Fahrenheit though |
| 58     | 8      | CRC over bits 0 - 57   | Truncated polynomial 19, init value 0 |

A trick I used in order to be able to use the Python crcmod to calculate this CRC was to prepend 6 dummy zero bits in front to make the message fit into complete bytes (won't affect CRC8 as long as padding is done in front)

The code will utilize the super handy HackRF One through GNURadio to either sniff existing sensors decoding the data above, or encode and transmit custom messages. This is a major Python hack, and Python is definitely not my primary choice of programming languages. The code here is built upon code that was generated by the GNU Radio Companion and I only added a custom source and sink respectively.

No warranties that this won't cause issues with your car, but it works for me. Transmit responsibly.
