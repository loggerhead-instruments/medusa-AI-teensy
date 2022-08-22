# medusa-AI-teensy

## Satellite Message Format Specification

### Message Limitations
Swarm has a limit of 192 characters per message

Iridium via Rockblock charges for every 50 characters. Max message 320 Bytes

### Message Format
Adopt modified JSON with reserved short one character names. These names have a known data type associated with them (e.g. string or number).

Each Swarm message ends with a NEMA checksum (e.g. *67) that is removed by the Swarm system.

Teensy will feed whatever string is stored by Coral in the detections.txt file into the satellite message. This will allow a user to change the Coral processing. The message format is designed to allow cloud processing of custom messages.

Teensy will deal with message packing to fit inside of message limits. A message may contain one or more JSON blocks separated by {braces}.

JSON strings will NOT be enclosed with “quotes”


### Reserved Names
```
Packet from Teensy
t: UNIX time stamp
i: file ID on message from Teensy
d: audio duration in s
a: lat
o: lon
b: band level sound
v: voltage (transmitted as int voltage * 10)
c: temperature (internal * 10) C
p: pressure internal hPa
h: humidity %rH
```

```
Packet from Coral
c: file ID on message from Coral (this will match a file ID on message from Teensy)
w: number of whistles
sE: error string added by Teensy if payload too long
```

example message assembled by Teensy that includes one Coral packet:
```
{t:1549317960,i:3A,a:26.3425,o:-82.3456,b:[72,72,72,43,54,34,98],v:39,c:256,p:1014,h:46}{c:3A,iW:3}*67
```

### Custom User Coral Messages
Users can use 2 or 3 character fields to pack any data they want.

The message should start with a data type specifier indicating whether it is text or a number.

lowercase i = number

lowercase s = string

e.g. for Right whale count the user could implement a detector on the Coral and output the results like:
```
{C:5B,iRW:5}
```

The maximum Coral payload including formatting characters is 50 bytes, unless the Teensy payload is modified to be smaller. The Coral payload will not be added to the message if it is too long.
