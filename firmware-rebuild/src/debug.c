    //__asm("bkpt");
      // debugging ADC below
      // DataPacket pk;
      //__asm("bkpt");
      //
      // int32_t val = MCP346x_analogRead(&extadc, subid << 1, (subid << 1) + 1,
      // GAIN_1); uprintf("%d\r\n", val); uprintf("%08x\r\n", val);
      // //__asm("bkpt");
      // pk.id = baseAddress | 0x5;
      // pk.err = 0;
      // pk.reserved = 0;
      // pk.reply = 0;
      // pk.datasize = 8;
      //
      // pk.data.bytes[0] = (uint8_t)val;
      // pk.data.bytes[1] = (uint8_t)(val >> 8);
      // pk.data.bytes[2] = (uint8_t)(val >> 16);
      // pk.data.bytes[3] = (uint8_t)(val >> 24);
      // pk.data.bytes[4] = 0x99;
      // pk.data.bytes[5] = 0x99;
      //
      // pk.data.cmd = BUSCMD_READ_ID_HIGH;
      // writeDatapacketToCan(&pk);
      // // can debug below
      // // DataPacket dp;
      // // dp.id = baseAddress | 0x5;
      // // dp.err = 0;                        // Not an error packet
      // // dp.reserved = 0;                   // Set to 0 for easier debugging
      // // dp.reply = 0;                      // Not a reply
      // // dp.data.seq = count & 0xFF;        // Increment sequence number each
      // // time dp.data.cmd = BUSCMD_READ_ID_HIGH; // Sets the ID to claim
      // // dp.data.bytes[0] = 0x00;
      // // dp.data.bytes[1] = 0x01;
      // dp.data.bytes[2] = 0x02;
      // dp.data.bytes[3] = 0x03;
      // dp.datasize = 6; // Include the sequence number and command in this

