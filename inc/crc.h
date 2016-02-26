union {
    uint32_t as_uint32;
    uint8_t as_4_uint8[4];
} crc;
void crc_init(void) {
    crc.as_uint32 = 0xffffffff;
}
void crc_update(uint8_t byte) {
    int8_t j;
    crc.as_uint32 = crc.as_uint32 ^ byte;
    for(j = 7; j >= 0; j--) {
        uint32_t mask = -((crc.as_uint32 & 1));
        crc.as_uint32 = (crc.as_uint32 >> 1) ^ (0xEDB88320 & mask);
    }
}
void crc_finalize(void) {
    crc.as_uint32 = ~crc.as_uint32;
}
