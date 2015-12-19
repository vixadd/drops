# ------------------------------------------------
# Generic Makefile
#
# Author: Nigel Armstrong
# Date  : 2015-11-02
#
# Changelog :
#   2010-11-05 - first version
#   2011-08-10 - added structure : sources, objects, binaries
#                thanks to http://stackoverflow.com/users/128940/beta
#   2015-03-30 - CNU UAS Lab version
#   2015-11-02 - Project Pulusan version
# ------------------------------------------------

# project name (generate executable with this name)
TARGET   = drops

CC       = g++
# compiling flags here
CFLAGS   = -Wall -I.

# linking flags here
LFLAGS   = -Wall -I. -lm -L /usr/local/lib -lsbpl

# change these to set the proper directories where each files shoould be
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
INCLUDE_DIR := /usr/local/include/sbpl
CFLAGS   += $(foreach includedir,$(INCLUDE_DIR),-I$(includedir))
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
rm       = rm -f

all: directories $(BINDIR)/$(TARGET)

$(BINDIR)/$(TARGET): $(OBJECTS)
  $(LINK.cc) $^ -o $@ $(LFLAGS)
  @echo "Linking complete!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp | $(OBJDIR)
  @$(CC) $(CFLAGS) -c $< -o $@
  @echo "Compiled "$<" successfully!"

directories:
  @mkdir -p $(BINDIR)
  @mkdir -p $(OBJDIR)

.PHONEY: clean
clean:
  @$(rm) $(OBJECTS)
  @echo "Cleanup complete!"

.PHONEY: remove
remove: clean
  @$(rm) $(BINDIR)/$(TARGET)
  @echo "Executable removed!"
