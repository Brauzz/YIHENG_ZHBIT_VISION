QT       += core gui printsupport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11 console
CTEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0



# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/local/include/opencv2/core
INCLUDEPATH += /usr/local/include/opencv2/highgui
INCLUDEPATH += /usr/local/include/opencv2/imgproc
INCLUDEPATH += /usr/local/include/opencv2/flann
INCLUDEPATH += /usr/local/include/opencv2/photo
INCLUDEPATH += /usr/local/include/opencv2/video
INCLUDEPATH += /usr/local/include/opencv2/features2d
INCLUDEPATH += /usr/local/include/opencv2/objdetect
INCLUDEPATH += /usr/local/include/opencv2/calib3d
INCLUDEPATH += /usr/local/include/opencv2/ml
INCLUDEPATH += /usr/local/include/opencv2/contrib
LIBS += `pkg-config opencv --cflags --libs`


#LIBS += -lgxiapi \
#-ldximageproc

#INCLUDEPATH += ../dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/library/CPP/include
#INCLUDEPATH += ../dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/include


HEADERS += \
    armor_detection/armor_detect.h \
    buff_detection/buff_detect.h \
    camera/camera_device.h \
    common/filter/predict.h \
    common/serial/serial_port.h \
    common/solve_angle/solve_angle.h \
    common/thread_control.h \
    mainwindow.h \
    qcustomplot.h \
    base.h \
    camera/save_video.h

SOURCES += \
    armor_detection/armor_detect.cpp \
    buff_detection/buff_detect.cpp \
    camera/camera_device.cpp \
    common/filter/predict.cpp \
    common/serial/serial_port.cpp \
    common/solve_angle/solve_angle.cpp \
    common/main.cpp \
    common/thread_control.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    camera/save_video.cpp

FORMS += \
    mainwindow.ui

DISTFILES += \
    camera/camera_param/cameraParam_0.xml \
    camera/camera_param/camera.xml \
    camera/camera_param/camera4mm.xml \
    camera/camera_param/camera4mm_2.xml \
    camera/camera_param/camera4mm_3.xml \
    camera/camera_param/camera4mm_5.xml \
    camera/camera_param/camera8mm.xml \
    camera/camera_param/camera8mm_1.xml \
    camera/camera_param/camera8mm_11.xml \
    camera/camera_param/galaxy_0.xml \
    camera/camera_param/galaxy_1.xml \
    camera/camera_param/galaxy_1024.xml \
    camera/camera_param/galaxy_2.xml \
    camera/camera_param/galaxy_3.xml

SUBDIRS += \
    galaxy/galaxy.pro
