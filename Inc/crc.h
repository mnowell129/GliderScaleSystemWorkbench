
#ifndef CRC_H
#define CRC_H

#define NOMINAL_CRC_SEED  0xFFFF

Word16 computeCRC(Byte *buffer,Int32 length,Word16 startingValue,Byte doLSB);
Word16 calcCRC(Byte *buffer, Int32 length);
Word16 calcCRCUnlimited(Byte *buffer, Int32 length);
void addCRC16(Byte *buffer,Int32 length);
Word16 runningCRC(Byte *buffer,Int32 length, Word16 partialCRC);
extern const Byte flip[];
Word16 calcCRCLSB(Byte *buffer, Int32 length);

#endif

