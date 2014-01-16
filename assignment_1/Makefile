CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

OBJS =		main.o parser.o

LIBS =		-losg -losgText -lOpenThreads -losgDB -losgUtil -losgGA -losgViewer -losgWidget

TARGET =	exercise

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
