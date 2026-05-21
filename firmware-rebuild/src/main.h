#define BUSCMD_READ_ID_LOW  0x10
#define BUSCMD_READ_ID_HIGH 0x11
#define BUSCMD_READ_VALUE   0x80
#define BUSCMD_WRITE_VALUE  0xC0
#define BUSCMD_CLAIM_ID     0x0F

#define FRAME_ERROR_ERROR 1
#define FRAME_SIZE_ERROR 2
#define FRAME_ID_ERROR 3

void onFatalError(void);
extern int fatal;
