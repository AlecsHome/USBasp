//Functions for sw microwire interface
void mwBegin(void);

void mwStart(void);

void mwSendData(uint16_t data, uint8_t bits);

unsigned char mwReadByte();

void mwEnd();

uchar mwBusy();

unsigned char mwGetAdrLen();

void mwReadDummyBit(void);