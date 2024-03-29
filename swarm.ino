// Swarm Tile Modem
#define maxTileChar 256
char tileStream[maxChar];
int tileStreamPos;

void pollTile(){
//  $DT 20210316170104,V*4c
//  $GN 27.2594,-82.4798,-3,0,2*1f

  while(Serial1.available()){
    byte incomingByte = Serial1.read();
    parseTile(incomingByte);
    Serial.write(incomingByte);
//    if(incomingByte=='$') display.println();
//    else display.print((char) incomingByte);
//    display.display();
  }
}

void parseTile(byte incomingByte){
  // check for start of new message
  // if a $, start it at Pos 0, and continue until next $
  if(incomingByte=='$') {
    //process last message
    if(tileStreamPos > 10){
      // extract RSSI
      if((tileStream[1]=='R') & (tileStream[2]=='T')){
       char temp[streamPos + 1];
       // two digit values
        if(tileStream[12]=='*') {
          memcpy(&temp, &tileStream[9], 3);
          Serial.print("RSSI extracted:"); Serial.println(temp);
          sscanf(temp, "%i", &rssi);
        }
        // three digit values
        if(tileStream[13]=='*'){
          memcpy(&temp, &tileStream[9], 4);
          Serial.print("RSSI extracted:"); Serial.println(temp);
          sscanf(temp, "%i", &rssi);
        }
      }
    }
    // start new message here
    tileStreamPos = 0;
  }
  tileStream[streamPos] = incomingByte;
  tileStreamPos++;
  if(tileStreamPos >= maxTileChar) tileStreamPos = 0;
}
