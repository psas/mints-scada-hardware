# Module Command Reference

## 0x block - Super important issues
| CMD  | Name         | Description
|------|--------------|-----------------------------------------------------------------------------------
| 0x00 | ESTOP        | Puts every module into a safe state. Ensure REPLY, ERR, and RES are all 0 to ensure it has top bus priority

## 1x block - Device Info
| CMD  | Name         | Description
|------|--------------|-----------------------------------------------------------------------------------
| 0x10 | READ_ID_LOW  | Reads the lower 6 bytes of the unique microcontroller ID
| 0x11 | READ_ID_HIGH | Reads the higher 6 bytes of the unique microcontroller ID
| 0x1F | ID_CLAIM     | Claims an ID for the module. Triggers an error reply if the ID is already claimed

## 8x block - Reading commands

## Cx block - Writing commands