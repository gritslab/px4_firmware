#
# Main system state machine
#

MODULE_COMMAND = commander
SRCS = commander_main.cpp \
	   commander.cpp \
	   led.cpp


MODULE_STACKSIZE = 1200

MAXOPTIMIZATION	 = -Os
