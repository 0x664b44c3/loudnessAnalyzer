#pragma once
#include <stdint.h>
enum FontAttr
{
    fntFont8x8    = 0x00,
    fntFont5x7    = 0x01,
    fntFont8x16   = 0x02,
    fntFont12x16  = 0x03,
    fntFont10x16  = 0x04,
    fntFont14x24  = 0x05,



    fntTypeFaceMask=0x0f,
    fntDoubleWide = 0x10,
    fntDoubleHigh = 0x20,
    fntDoubleSize = 0x30,
    fntUnderline  = 0x40,
    fntInvert     = 0x80,

    fntFontDefault = fntFont8x8

};

void drawText(int x, int y, uint8_t fontAttr, const char * text, int DTS);
int drawGlyph(int x, int y, uint8_t fontAttr, unsigned char c, int DTS);

struct fontInfo {
    uint8_t  chWidth;
    uint8_t  chHeight;
    uint8_t  chAddSpace;
    uint16_t numGlyphs;
    uint8_t  glyphOffset;
    uint8_t  underlineY;
    char *   fontData;
};

extern struct fontInfo fonts[];
extern const int NUM_FONTS;
