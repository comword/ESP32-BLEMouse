#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := hidd_mouse

COMPONENT_SRCDIRS = . Invn Invn/Devices Invn/Devices/Drivers/Icm207xx

COMPONENT_ADD_INCLUDEDIRS := components/include	\

include $(IDF_PATH)/make/project.mk
