# Compiler
#
CC = g++

# Compiler flags
#
CC_FLAGS = -std=c++11 -Wall -g 

# Include directories
#
INCLUDES = -I/usr/include/python2.7

# Linker flags
#
LINK_FLAGS = 

# Libraries
#
LIBS = -lpthread -lncurses -lpython2.7 -lcurl
# -lduma

# Build directorys
#
OBJDIR = obj
TARGETDIR = bin

# Executable name
#
EXEC = $(TARGETDIR)/tk_gcs 

# Source and object files
#
SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.cpp=.o))

$(EXEC): $(OBJECTS)
	@echo linking $(EXEC)
	@mkdir -p $(@D)
	$(CC) $(LFLAGS) $(OBJECTS) $(LIBS) -o $(EXEC)

$(OBJDIR)/%.o: %.cpp
	@echo compiling $<
	@mkdir -p $(@D)
	$(CC) -c $(CC_FLAGS) $< -o $@

clean:
	@echo cleaning project
	rm -f $(EXEC) $(OBJECTS)

install:
	@echo installing tk_gcs file
	rm /home/pi/hti/apps/fms/tk_gcs
	cp $(EXEC) /home/pi/hti/apps/fms

