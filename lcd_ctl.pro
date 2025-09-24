QT-= core gui

include(esp_includes.pri)


include($$PWD/components/display/display.pri)
include($$PWD/components/fonts/fonts.pri)
include($$PWD/managed_components/espressif__esp-dsp/esp-dsp.pri)

DISTFILES += \
    CMakeLists.txt \
    main/component.mk \
    main/CMakeLists.txt \
    esp_includes.pri

SOURCES += \
    main/dsp.c \
    main/main.c \
    main/terminal.c

HEADERS += \
    components/bitmaps/ef_wolf.h \
    main/bootscreen.h \
    main/dsp.h


DEFINES+=portTICK_PERIOD_MS=10 configNUM_CORES=2
DEFINES+=configUSE_CORE_AFFINITY=1

INCLUDEPATH+=$$PWD/managed_components/espressif__esp-dsp

#HEADERS +=
