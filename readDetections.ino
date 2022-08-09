// detections.txt 
// payload that will be appended to audio stats

boolean readDetections()
{
   FsFile detFile;

  if(detFile.open("detections.txt"))
  {
    int n = detFile.fgets(coralPayload, sizeof(coralPayload));
    if(n<=0){
      detFile.close();
      return 0;
    }
    detFile.close();  
  }
  else return 0;
  Serial.print("Detections:");
  Serial.println(coralPayload);

  // extract whistle count
  char signalLabel[1];
  unsigned int detectionCount;
  sscanf(coralPayload, "%c:%d",signalLabel,&detectionCount);
  if(signalLabel[0]=='w') {
    whistleCount = detectionCount;
    Serial.print("scan whistles:");
    Serial.println(whistleCount);
  }
 return 1;  
}
