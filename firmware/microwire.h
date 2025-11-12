//Functions for sw microwire interface
void mwBegin(void);

void mwStart(void);

uint8_t mwSendData(uint8_t data, uint8_t bits);

unsigned char mwReadByte();

void mwEnd();

uchar mwBusy();

unsigned char mwGetAdrLen();
