#include <dmptext.h>
#include <i2c.h>
#include <math.h>


#define M_PI 3.141592654





code unsigned char dmpmemorydata[1929]={  
 // bank 0, 256 bytes  
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,  
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,  
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,  
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,  
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,  
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,  
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,  
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,  
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,  
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,  
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,  
    // bank 1, 256 bytes  
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,  
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,  
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,  
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,  
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,  
    // bank 2, 256 bytes  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,  
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    // bank 3, 256 bytes  
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,  
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,  
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,  
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,  
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,  
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,  
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,  
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,  
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,  
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,  
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,  
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,  
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,  
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,  
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,  
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,  
    // bank 4, 256 bytes  
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,  
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,  
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,  
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,  
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,  
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,  
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,  
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,  
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,  
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,  
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,  
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,  
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,  
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,  
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,  
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,  
    // bank 5, 256 bytes  
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,  
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,  
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,  
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,  
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,  
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,  
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,  
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,  
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,  
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,  
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,  
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,  
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,  
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,  
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,  
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,  
    // bank 6, 256 bytes  
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,  
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,  
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,  
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,  
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,  
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,  
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,  
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,  
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,  
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,  
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,  
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,  
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,  
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,  
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,  
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,  
    // bank 7, 138 bytes (remainder)  
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,  
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,  
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,  
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,  
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,  
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,  
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,  
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,  
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF  
};  
code unsigned char dmpcfgupddata[192] = {  
//  dmp config   
//  BANK    OFFSET  LENGTH  [DATA]  
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,           
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,           
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,     
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,     
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,   
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,           
    0x00,   0x6C,   0x02,   0x20, 0x00,                 
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,    
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,    
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,    
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,     
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,    
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,  
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,           
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,     
    0x00,   0xA3,   0x01,   0x00,                      
    0x00,   0x00,   0x00,   0x01,   //这里是开启DMP的特殊中断的  
    //原程序中此行代码为(这里不一定错)  
    //0x00,   0x00,   0x00,   0x01,  即LENGTH=0x00，有错  
          
    0x07,   0x86,   0x01,   0xFE,                       
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38,   
    0x07,   0x7E,   0x01,   0x30,                     
    0x07,   0x46,   0x01,   0x9A,                      
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,     
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,    
    0x02,   0x16,   0x02,   0x00, 0x01,                
/* 上行最后一个数据调整FIFO rate :0x01=100HZ,0x02=66HZ,0x03=50HZ ,0x04=40HZ,0x05=33.33HZ,  
// 可从 datasheet 公式推算 
//dmp updates 
    0x01,   0xB2,   0x02,   0xFF, 0xFF, 
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35, 
    0x01,   0x6A,   0x02,   0x06, 0x00, 
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00, 
    0x01,   0x62,   0x02,   0x00, 0x00, 
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00*/  
};  
code unsigned char dmpUpdates[47]={  
      
    0x01,   0xB2,   0x02,   0xFF, 0xFF,  
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,  
    0x01,   0x6A,   0x02,   0x06, 0x00,  
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,  
    0x01,   0x62,   0x02,   0x00, 0x00,  
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00  
      
};  


void DelayUs2x(unsigned char t) //延时1  
{     
 while(--t);  
}  
  
void DelayMs(unsigned char t)   //延时2  
{  
       
 while(t--)  
 {  
     //大致延时1mS  
     DelayUs2x(245);  
     DelayUs2x(245);  
 }  
}  




 /* 
   加载 DMP代码到 
   返回值  (1=成功,0=失败) 
 */  
void loadfirmware(void)      
{  
  unsigned int datanum=0;   //DMP固件写入标志位  
  unsigned char ye,i,j;  
  unsigned char bank=0; //段（256个数据一段）  
  unsigned char addr=0;   
  
    for(;bank<8;bank++)  
    {  
        if(bank == 7)   //这里的作用就是区分最后一段数据  
            i = 8;  
        else  
            i = 16;  
        for(ye=0;ye<i;ye++)  
        {  
            send_I2c(0x6d,bank);  
            send_I2c(0x6e,addr);  
            start_I2c();    //起始信号  
            send_bite(SlaveAddressW);    //设备地址+写信号  
					  Wake_I2c();
            send_bite(0x6f);    //设备内部地址  
            for(j=0;j<16;j++)  
            {  
                send_bite(dmpmemorydata[datanum++]);    //写入DMP的数据  
							  Wake_I2c();
         //       if(ack) return 0;   
            }  
            addr += 16;  
            stop_I2c();    //停止信号  
        }  
    }  
    send_I2c(0x6d,7);  
    send_I2c(0x6e,addr);  
    start_I2c();    //起始信号  
    send_bite(SlaveAddressW);    //设备地址+写信号  
		Wake_I2c();
    send_bite(0x6f);    //设备内部地址  
		Wake_I2c();
    for(i=0;i<9;i++)  
    {  
        send_bite(dmpmemorydata[datanum++]);    //写入DMP的数据  
			  Wake_I2c();
   //     if(ack) return 0;  
    }  
    stop_I2c();    //停止信号  
 //   return 1;  
}  

void loadcfgupd(void)  //DMP设置  
{  
  unsigned char line;   //一共需要写入30条设置数据  
  unsigned char bank;   //页  
  unsigned char datacounts=0;   //DMP设置数据标志位  
  unsigned char bytes2write;    //数据长度。  
  unsigned char offset; //偏移地址  
  unsigned char writingcounts;  //数据写入标志与bytes2write一同使用  
  unsigned char special;  
       
  for (line=0;line<30;line++)  
  {  
    bank=dmpcfgupddata[datacounts++];  
    offset=dmpcfgupddata[datacounts++];  
    bytes2write=dmpcfgupddata[datacounts++];  
    send_I2c(0x6d,bank);  
    send_I2c(0x6e,offset);  
    start_I2c();    //起始信号  
    send_bite(SlaveAddressW);    //设备地址+写信号  
		Wake_I2c();
    send_bite(0x6f);    //设备内部地址  
		Wake_I2c();
    for(writingcounts=0;writingcounts<bytes2write;writingcounts++)  
    {  
        send_bite(dmpcfgupddata[datacounts++]); //写入DMP配置数据  
			  Wake_I2c();
   //         if(ack) return 0;  
    }  
    if(0 == bytes2write)  
    {  
        special=dmpcfgupddata[datacounts++];  
        if(0x01 == special)  
            {  
                //设置零运动中断启用（真）;  
          //设置FIFO缓冲区溢出启用（真）;  
          //设置DMP启用（真）;  
                send_I2c(0x38,0x32);  
            }  
  //      else  
   //         return 0;  
    }  
  }  
  stop_I2c();  //停止信号  
 //   return 1;  
}  
/*最后更新DMP*/  
void xdmpUpdates(unsigned char datacounts)  
{  
    unsigned char writingcounts,bank,offset,bytes2write;  
    bank=dmpUpdates[datacounts++];  
  offset=dmpUpdates[datacounts++];  
  bytes2write=dmpUpdates[datacounts++];  
  send_I2c(0x6d,bank);  
  send_I2c(0x6e,offset);  
  start_I2c();  //起始信号  
  send_bite(SlaveAddressW);    //设备地址+写信号  
	Wake_I2c();
  send_bite(0x6f);    //设备内部地址  
	Wake_I2c();
    for(writingcounts=0;writingcounts<bytes2write;writingcounts++)  
    {  
        send_bite(dmpUpdates[datacounts++]);    //写入DMP配置数据  
			  Wake_I2c();
  //          if(ack) return 0;  
    }  
    stop_I2c();    //停止信号  
 //   return 0;  
}  
/*读取 FIFO 计数*/  
unsigned int getFIFOCount()  
{  
    unsigned char i[2];  
    recs_I2c(0x72,2,i);  
    return ((i[0]<<8)+i[1]);  
}  
/*FIFO数据读取 
参数 *Data    存储数据的地址 
返回值 (1=读取成功,0读取失败) 
*/  
void readdmp(unsigned char *Data)  
{  
 recs_I2c(0x74,42,Data);  
}  


void dmpInitialize(void)  
{  
    unsigned char hwRevision,otpValid,mpuIntStatus/*fifoBuffer[128]*/;  
    unsigned char xgOffsetTC,ygOffsetTC,zgOffsetTC;  
    unsigned int fifoCount;  
    writeBit(0x6B,7,1); //复位 MPU6050  
    DelayMs(50);
    writeBit(0x6B,6,0); //禁止睡眠模式  
    send_I2c(0x6D,0x70);    //写入一个字节数据到0x6d寄存器(选择用户 bank)  
    send_I2c(0x6E,0x06);    //写入一个字节数据到0x6e寄存器(选择存储字节)  
    recs_I2c(0x6F,1,&hwRevision);  //读取  
    send_I2c(0x6D,0);   //重置内存 bank 选择  
    readBit(0x00,0,&otpValid);  //读取 OTP bank 有效标志  
    readBits(0x00,6,6,&xgOffsetTC); //读陀螺偏置TC值 X  
    readBits(0x01,6,6,&ygOffsetTC); //读陀螺偏置TC值 Y)  
    readBits(0x02,6,6,&zgOffsetTC); //读陀螺偏置TC值 Z  
    send_I2c(0x25,0x7f);    //设置从0地址 0x7  
    writeBit(0x6A,5,0); //禁用I2C主模式  
    send_I2c(0x25,0x68);    //这里可能要改。还没有弄明白这里  
    writeBit(0x6A,1,1); //I2C总线主控复位  
    DelayMs(20);  
    loadfirmware();   //加载 DMP代码到内存  
    loadcfgupd();  //配制DMP  
    writeBits(0x6B,2,3,0x03);   //设置时钟脉冲源Z陀螺  
    send_I2c(0x38,0x12);    //设置DMP和FIFO_OFLOW启用中断  
    send_I2c(0x19,4);   //设置采样率为200 hz  (1khz / (1 + 4) = 200 Hz)  
    writeBits(0x1A,5,3,0x1);    //设置外部帧同步TEMP_OUT_L[0]  
    writeBits(0x1A,2,3,0x03);   //设置DLPF带宽42赫兹  
    writeBits(0x1B,4,2,0x03);   //陀螺灵敏度设置为+ / - 2000 deg/sec  
    send_I2c(0x70,0x03);    //设置DMP配置字节（功能未知）  
    send_I2c(0x71,0x00);    //设置DMP配置字节（功能未知）  
    writeBit(0x00,0,0); //清除OTP Bank 标志  
    writeBits(0x00,6,6,xgOffsetTC); //设置X 陀螺抵消TCs之前的值  
    writeBits(0x01,6,6,ygOffsetTC); //设置Y 陀螺抵消TCs之前的值  
    writeBits(0x02,6,6,zgOffsetTC); //设置Z 陀螺抵消TCs之前的值  
    xdmpUpdates(0); //最后更新1/7(函数未知)dmpUpdates数组第一行  
    xdmpUpdates(5); //最后更新2/7(函数未知)dmpUpdates数组第二行  
    writeBit(0x6A,2,1); //复位 FIFO  
    fifoCount = getFIFOCount(); //读取 FIFO 计数  
    //readdmp(fifoCount,fifoBuffer);    //读取FIFO里的数据  
    writeBit(0x6A,2,1); //复位 FIFO  
      
    send_I2c(0x1F,2);   //运动检测阈值设置为2  
    send_I2c(0x21,156); //零运动检测阈值为156  
    send_I2c(0x20,80);  //设置运动检测持续时间至80  
    send_I2c(0x22,0);   //设置零运动检测时间0  
    writeBit(0x6A,2,1); //复位 FIFO  
    writeBit(0x6A,6,1); //使能 FIFO  
    writeBit(0x6A,7,1); //使能 DMP  
    writeBit(0x6A,3,1); //复位 DMP  
    xdmpUpdates(12);    //最后更新3/7(函数未知)dmpUpdates数组第三行  
    xdmpUpdates(17);    //最后更新4/7(函数未知)dmpUpdates数组第四行  
    xdmpUpdates(28);    //最后更新5/7(函数未知)dmpUpdates数组第五行  
    while((fifoCount = getFIFOCount()) < 3); //等待 FIFO 计数 > 2  
    writeBit(0x6A,2,1); //复位 FIFO  
    //readdmp(fifoCount,fifoBuffer);    //读取FIFO里的数据  
    recs_I2c(0x3A,1,&mpuIntStatus);    //读取中断状态  
    xdmpUpdates(35);    //最后更新6/7(函数未知)dmpUpdates数组第六行  
    while((fifoCount = getFIFOCount()) < 3); //等待 FIFO 计数 > 2  
    writeBit(0x6A,2,1); //复位 FIFO  
    //readdmp(fifoCount,fifoBuffer);    //读取FIFO里的数据  
    recs_I2c(0x3A,1,&mpuIntStatus);    //读取中断状态  
    xdmpUpdates(40);    //最后更新7/7(函数未知)dmpUpdates数组第七行  
    writeBit(0x6A,7,0); //禁用DMP(稍后您打开它)  
    writeBit(0x6A,2,1); //复位 FIFO  
    recs_I2c(0x3A,1,&mpuIntStatus);  
    //星期六 (2014/06/28)   
}  
/*********compute quaters *************/

void compute (void)
{
	unsigned char iii,qq[4],q[4],pitch,roll,yaw,norm,zd;
	    iii=getFIFOCount();//读取FIFO计数  
    recs_I2c(0x3A,1,&zd);  //读取中断状态  
    if((zd & 0x10)||iii==1024)    //判断FIFO是否溢出  
    {  
        writeBit(0x6A,2,1); //复位 FIFO  
    }  
    else if (zd & 0x02)  
    {  
        while(iii<42) iii=getFIFOCount();  
        readdmp(dmpdatas);  //读取FIFO数据(四元数+其他的数据)  
        qq[0]=(dmpdatas[0]);  
        qq[1]=(dmpdatas[2]);  
        qq[2]=(dmpdatas[4]);   
        qq[3]=(dmpdatas[6]);   
				norm = sqrt(qq[0]*qq[0] + qq[1]*qq[1] + qq[2]*qq[2] + qq[3]*qq[3]);
	      q[0] = qq[0] * norm;
	      q[1] = qq[1] * norm;
	      q[2] = qq[2] * norm;
	      q[3] = qq[3] * norm;
	
	      roll = (atan2(2.0*(q[0]*q[1] + q[2]*q[3]),
	                       1 - 2.0*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;
	 // we let safe_asin() handle the singularities near 90/-90 in pitch
	      pitch = asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;
	//??:??????,?????
	      yaw = -atan2(2.0*(q[0]*q[3] + q[1]*q[2]),
	                     1 - 2.0*(q[2]*q[2] + q[3]*q[3]))* 180/M_PI;
    } 
}