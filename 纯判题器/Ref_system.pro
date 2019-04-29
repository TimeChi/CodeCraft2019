TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    answer.cpp \
    car.cpp \
    cross.cpp \
    data_pre_processing.cpp \
    main.cpp \
    road.cpp \
    presetanswer.cpp \
    referee_system.cpp

SUBDIRS += \
    Ref_system.pro

DISTFILES += \
    Ref_system.pro.user

HEADERS += \
    answer.h \
    car.h \
    cross.h \
    data_pre_processing.h \
    road.h \
    presetanswer.h \
    referee_system.h
