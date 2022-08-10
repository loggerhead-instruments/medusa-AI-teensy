# medusa-AI-teensy

## Satellite Message Format Specification

### Message Limitations
Swarm has a limit of 192 characters per message
Iridium via Rockblock charges for every 50 characters. Max message 320 Bytes

### Message Format
Adopt modified JSON with reserved short one character names. These names have a known data type associated with them (e.g. string or number).

Each message ends with a NEMA checksum (e.g. *67) to ensure data integrity

Teensy will feed whatever string is stored by Coral in the detections.txt file into the satellite message. This will allow a user to change the Coral processing and us to create a web system to process these messages.

Teensy will deal with message packing to fit inside of message limits. A message may contain one or more JSON blocks separated by {braces}.

JSON strings will NOT be enclosed with “quotes”


### Reserved Names
```
Packet from Teensy
t: UNIX time stamp
i: file ID on message from Teensy
a: lat
o: lon
b: band level sound
```
### Packet from Coral
```
c: file ID on message from Coral (this will match a file ID on message from Teensy)
w: number of whistles
```

example:
```
{t:1549317960,i:3A,a:26.3425,o:-82.3456,b:[72,72,72,43,54,34,98]}{c:3A,w:3}*67
```

Users can use 2 or 3 character fields to pack any data they want.
lowercase i = number
lowercase s = string
e.g. for Right whale count the user could implement a detector on the Coral and output the results like:
{c:5B,RWi:5}
