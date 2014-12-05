GCC = g++
TARGET = TestExe
CFLAGS = -pthread -lm

all:$(TARGET)

$(TARGET):database.cpp CalSelfLoc.h calcoordinates.h axialMove.h SendData.h
	$(GCC) $(CFLAGS) -o $@ $^

clean:
	rm -r $(TARGET)