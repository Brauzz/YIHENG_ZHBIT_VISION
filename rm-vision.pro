QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
    image_produce_process.cpp \
    capture_video.cpp \
    armor_detect.cpp \
    solve_angle.cpp \
    serial_port.cpp \
    predict.cpp \
    buff_detect.cpp

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



LIBS += /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_objdetect.so.3.4.6 \
        /usr/local/lib/libopencv_calib3d.so.3.4 \
        /usr/local/lib/libopencv_photo.so \
        /usr/local/lib/libopencv_calib3d.so.3.4.6 \
        /usr/local/lib/libopencv_photo.so.3.4 \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_photo.so.3.4.6 \
        /usr/local/lib/libopencv_core.so.3.4  \
        /usr/local/lib/libopencv_shape.so \
        /usr/local/lib/libopencv_core.so.3.4.6 \
        /usr/local/lib/libopencv_shape.so.3.4 \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_shape.so.3.4.6 \
        /usr/local/lib/libopencv_features2d.so.3.4 \
        /usr/local/lib/libopencv_stitching.so \
        /usr/local/lib/libopencv_features2d.so.3.4.6 \
        /usr/local/lib/libopencv_stitching.so.3.4 \
        /usr/local/lib/libopencv_flann.so \
        /usr/local/lib/libopencv_stitching.so.3.4.6 \
        /usr/local/lib/libopencv_flann.so.3.4 \
        /usr/local/lib/libopencv_superres.so \
        /usr/local/lib/libopencv_flann.so.3.4.6 \
        /usr/local/lib/libopencv_superres.so.3.4 \
        /usr/local/lib/libopencv_superres.so.3.4.6 \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_highgui.so.3.4 \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_highgui.so.3.4.6 \
        /usr/local/lib/libopencv_videoio.so.3.4 \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/local/lib/libopencv_videoio.so.3.4.6 \
        /usr/local/lib/libopencv_imgcodecs.so.3.4 \
        /usr/local/lib/libopencv_video.so \
        /usr/local/lib/libopencv_imgcodecs.so.3.4.6 \
        /usr/local/lib/libopencv_video.so.3.4 \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_video.so.3.4.6 \
        /usr/local/lib/libopencv_imgproc.so.3.4 \
        /usr/local/lib/libopencv_videostab.so \
        /usr/local/lib/libopencv_imgproc.so.3.4.6 \
        /usr/local/lib/libopencv_videostab.so.3.4 \
        /usr/local/lib/libopencv_ml.so \
        /usr/local/lib/libopencv_videostab.so.3.4.6 \
        /usr/local/lib/libopencv_ml.so.3.4 \
        /usr/local/lib/libopencv_ml.so.3.4.6 \
        /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_objdetect.so.3.4

HEADERS += \
    image_produce_process.h \
    capture_video.h \
    armor_detect.h \
    solve_angle.h \
    serial_port.h \
    predict.h \
    buff_detect.h \
    settings.h

FORMS +=
