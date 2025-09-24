#include <inttypes.h>
#include <string.h>
#include "font.h"

#include <stdio.h>
#include "font12x16.h"
#include "font8x8_basic.h"
#include "fnt8x16.h"
#include "font10x16.h"
#include "font14x24.h"
#include "fnt5x7.h"

#include "../display/display.h"

struct fontInfo fonts[] =
    {
        {
            .chWidth = 8,
            .chHeight= 8,
            .chAddSpace=0,
            .numGlyphs=128,
            .glyphOffset = 0,
            .fontData = (char*) font8x8_basic_tr
        },
        {
            .chWidth = 5,
            .chHeight= 8,
            .chAddSpace=1,
            .numGlyphs=256,
            .glyphOffset = 0,
            .underlineY = 8,
            .fontData = (char*) font5x7int
        },
        {
            .chWidth = 8,
            .chHeight= 16,
            .chAddSpace=0,
            .numGlyphs=256,
            .glyphOffset = 0,
            .underlineY =14,
            .fontData = (char*) font8x16
        },
        {
            .chWidth = 12,
            .chHeight= 16,
            .chAddSpace=0,
            .numGlyphs=64,
            .glyphOffset = ' ',
            .underlineY = 8,
            .fontData = (char*) largeFont
        },
        {
            .chWidth = 10,
            .chHeight= 16,
            .chAddSpace=0,
            .numGlyphs=256,
            .glyphOffset = 0,
            .underlineY = 8,
            .fontData = (char*) font10x16
        },
        {
            .chWidth = 14,
            .chHeight= 24,
            .chAddSpace=0,
            .numGlyphs=96,
            .glyphOffset = ' ',
            .underlineY = 24,
            .fontData = (char*) font14x24
        }
};

//struct fontInfo * currentFont;
const int NUM_FONTS = 6;

// static uint8_t masks[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
// void setPx(int x, int y, char onOff)
// {
//     if ((x<0)||(x>127)||(y<0)||(y>31))
//         return;
//     if (onOff)
//         __oled_frameBuffer[(y / 8) * FB_WIDTH + x]|=masks[y&7];
//     else
//         __oled_frameBuffer[(y / 8) * FB_WIDTH + x]&=~masks[y&7];
// }

int drawGlyph(int x, int y, uint8_t fontAttr, unsigned char c, int DTS)
{
    struct fontInfo *fi = &fonts[fontAttr & fntTypeFaceMask];
    int glyphId = c;
    glyphId -= fi->glyphOffset;
    if ((glyphId<0) || (glyphId >= fi->numGlyphs))
    {
        return (fi->chWidth + fi->chAddSpace) * ((fontAttr & fntDoubleWide)?2:1);
    }

    int nStripes = (fi->chHeight + 7) / 8;
    int u=0;
    for (int r=0;r<nStripes;++r)
    {
        int dp = fi->chWidth * glyphId;
        if (r) {
            dp += fi->chWidth * fi->numGlyphs * r;
            u=0;
        }

        char  *glyphData = &fi->fontData[dp];

        for (int dx=0;dx<fi->chWidth;++dx)
        {
            char  bits = *glyphData++;
            if (fontAttr & fntInvert)
                bits^=0xff;
            for(int d=0;d<((fontAttr&fntDoubleWide)?2:1);++d)
            {
                if (fontAttr & fntDoubleHigh) {
                    for (int v=0;v<8;++v)
                    {
                        if (DTS)
                        {
                            _setPx(x+u, y+(v+r*8)*2, bits&(1<<v));
                            _setPx(x+u, y+(v+r*8)*2+1, bits&(1<<v));
                        }
                        else
                        {
                            setPx(x+u, y+(v+r*8)*2, bits&(1<<v));
                            setPx(x+u, y+(v+r*8)*2+1, bits&(1<<v));
                        }
                    }
                }
                else
                {
                    for (int v=0;v<8;++v)
                    {
                        if (DTS)
                            _setPx(x+u, y+v+r*8, bits&(1<<v));
                        else
                            setPx(x+u, y+v+r*8, bits&(1<<v));
                    }
                }
                ++u;
            }
        }
    }
    for(int i=0;i<fi->chAddSpace;++i)
    {
        for (int v=0;v<fi->chHeight * ((fontAttr & fntDoubleWide)?2:1);++v)
            if (DTS)
                _setPx(x+u, y+v, fontAttr & fntInvert);
            else
                setPx(x+u, y+v, fontAttr & fntInvert);
        ++u;
    }
    return u;

}
void drawText(int x, int y, uint8_t fontAttr, const char * text, int DTS)
{
    uint8_t font = fontAttr & fntTypeFaceMask;
    if (font>=NUM_FONTS)
        font = NUM_FONTS-1;

    const char * td = text;

    unsigned char c = (unsigned char) *td++;
    struct fontInfo *fi = &fonts[font];

    int u=0;
    while (c)
    {
        u+=drawGlyph(x+u,y,fontAttr,c,DTS);
        c = (unsigned char) *td++;
    }
}


// void displayMnemonic(const char* txt, int dxw)
// {
// 	drawText(8, 10, 5, txt, 0);//| (dxw)?fntDoubleWide:0, txt, 0);
// }

// void displayText(int pos, const char *text) {
// 	int ptr = pos*8;
// 	while(*text)
// 	{
// 		unsigned cc = *text++;
// 		for (int i=0;i<8;++i)
// 		{
// 			__oled_frameBuffer[ptr++] = font8x8_basic_tr[cc&127][i];
// 		}
// 	}
// }
