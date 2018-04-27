#-------------------------------------------------
#
# Project created by QtCreator 2018-02-23T10:54:03
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = rrt_planning_DC
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    gen_new_node.cpp \
    plots.cpp \
    gnuplot_ci.cpp

HEADERS += \
    gen_new_node.h \
    gnuplot_ci.h
