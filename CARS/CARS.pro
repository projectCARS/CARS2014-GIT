#-------------------------------------------------
#
# Project created by QtCreator 2014-07-10T10:07:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CARS
TEMPLATE = app
# Include path for OpenCV.
INCLUDEPATH += C:\\OpenCV2.4.9\\opencv\\build\\include
# Include path for Flycapture2.
INCLUDEPATH += "C:\\Program Files\\Point Grey Research\\FlyCapture2\\include"
# Include path for Eigen.
INCLUDEPATH += C:\\eigen-eigen-6b38706d90a9 #\\eigen-eigen-6b38706d90a9

# Library files for OpenCV.
CONFIG( debug, debug|release ) {
LIBS += -LC:\\OpenCV2.4.9\\opencv\\build\\x64\\vc12\\lib \
-lopencv_core249d \
-lopencv_highgui249d \
-lopencv_imgproc249d \
-lopencv_features2d249d \
-lopencv_calib3d249d \
-lopencv_nonfree249d
} else {
LIBS += -LC:\\OpenCV2.4.9\\opencv\\build\\x64\\vc12\\lib \
-lopencv_core249 \
-lopencv_highgui249 \
-lopencv_imgproc249 \
-lopencv_features2d249 \
-lopencv_calib3d249d \
-lopencv_nonfree249d
}

# Include path for threading building blocks
INCLUDEPATH += "C:\\tbb43_20141023oss\\include"
LIBS += -LC:\\tbb43_20141023oss\\lib\\ia32\\vc12 \
-ltbb \
-ltbbmalloc \
-ltbbmalloc_proxy

#LIBS += -ltbb\
#        -ltbbmalloc \
#        -ltbbmalloc_proxy

# Library files for FlyCapture2.
CONFIG( debug, debug|release ) {
LIBS += -L"C:\\Program Files\\Point Grey Research\\FlyCapture2\\lib64" \
-lFlyCapture2d
} else {
LIBS += -L"C:\\Program Files\\Point Grey Research\\FlyCapture2\\lib64" \
-lFlyCapture2
}

# Library file for FlyCapture2.
LIBS += -LC:\\Users\\User\\Documents\\QtProjects\\CARS \
-lNIDAQmx

SOURCES += main.cpp\
        mainwindow.cpp \
    processingthread.cpp \
    controllerthread.cpp \
    referencedialog.cpp \
    cargroupbox.cpp \
    carsettingsdialog.cpp \
    IOControl.cpp \
    Car.cpp \
    CTModel.cpp \
    EKF.cpp \
    functions.cpp \
    PGRCamera.cpp \
    PIDController.cpp \
    VirtualSensor.cpp \
    AutoReverse.cpp \
    NoFilter.cpp \
    drawsettingsdialog.cpp \
    Calibrator.cpp \
    PIDControllerSR.cpp \
    particleFilter.cpp

HEADERS  += mainwindow.h \
    processingthread.h \
    controllerthread.h \
    referencedialog.h \
    cargroupbox.h \
    carsettingsdialog.h \
    definitions.h \
    qtheaders.h \
    headers.h \
    classes.h \
    Car.h \
    Controller.h \
    CTModel.h \
    EKF.h \
    Filter.h \
    functions.h \
    IOControl.h \
    MotionModel.h \
    PGRCamera.h \
    PIDController.h \
    VirtualSensor.h \
    AutoReverse.h \
    NoFilter.h \
    drawsettingsdialog.h \
    Calibrator.h \
    PIDControllerSR.h \
    ParticleFilter.h

FORMS    += mainwindow.ui \
    referencedialog.ui \
    cargroupbox.ui \
    carsettingsdialog.ui \
    drawsettingsdialog.ui

#RESOURCES += \
#    CARS.qrc
