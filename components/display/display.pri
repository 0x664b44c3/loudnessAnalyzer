INCLUDEPATH+=$$PWD

SOURCES += \
    $$PWD/display.c \
    $$PWD/i2s_parallel.c
HEADERS += \
    components/display/display.h \
    components/display/i2s_parallel.h

DISTFILES += \
    $$PWD/CMakeLists.txt
