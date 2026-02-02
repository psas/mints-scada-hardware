#define BUSCMD_ESTOP        0x00

#define BUSCMD_READ_ID_LOW  0x10
#define BUSCMD_READ_ID_HIGH 0x11
#define BUSCMD_READ_VALUE   0x80
#define BUSCMD_WRITE_VALUE  0xC0
#define BUSCMD_CLAIM_ID     0x0F

#define PACKET_ERROR_ERROR 1
#define PACKET_SIZE_ERROR 2
#define PACKET_ID_ERROR 3

int main(void);
int calc_baseAddress(void);
