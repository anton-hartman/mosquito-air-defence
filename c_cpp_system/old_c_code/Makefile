DIR_DETECTION = ./detection
DIR_STEPPERS = ./steppers
DIR_BIN = ./bin

OBJ_C = $(wildcard ${DIR_DETECTION}/*.c ${DIR_STEPPERS}/*.c ./*.c)
OBJ_O = $(patsubst %.c,${DIR_BIN}/%.o,$(notdir ${OBJ_C}))

TARGET = main
#BIN_TARGET = ${DIR_BIN}/${TARGET}

CC = gcc

DEBUG = -g -O0 -Wall
CFLAGS += $(DEBUG) 

LIB = 

${TARGET}:${OBJ_O}
	$(CC) $(CFLAGS) $(OBJ_O) -o $@  $(LIB)

${DIR_BIN}/%.o : ./%.c
	$(CC) $(CFLAGS) -c  $< -o $@  $(LIB)

${DIR_BIN}/%.o : $(DIR_STEPPERS)/%.c
	$(CC) $(CFLAGS) -c  $< -o $@  $(LIB)

${DIR_BIN}/%.o:$(DIR_DETECTION)/%.c
	$(CC) $(CFLAGS) -c  $< -o $@  $(LIB)
	
clean :
	rm $(DIR_BIN)/*.* 
	rm $(TARGET) 

run: ${TARGET}
	./${TARGET}