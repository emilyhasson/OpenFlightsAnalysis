EXENAME = main
OBJS =  main.o Graph.o

CXX = clang++
CXXFLAGS = -std=c++1y -stdlib=libc++ -c -g -O0 -Wall -Wextra -pedantic
LD = clang++
LDFLAGS = -std=c++1y -stdlib=libc++ -lc++abi -lm

# Custom Clang version enforcement logic:
ccred=$(shell echo -e "\033[0;31m")
ccyellow=$(shell echo -e "\033[0;33m")
ccend=$(shell echo -e "\033[0m")

IS_EWS=$(shell hostname | grep "ews.illinois.edu")
IS_CORRECT_CLANG=$(shell clang -v 2>&1 | grep "version 6")
ifneq ($(strip $(IS_EWS)),)
ifeq ($(strip $(IS_CORRECT_CLANG)),)
CLANG_VERSION_MSG = $(error $(ccred) On EWS, please run 'module load llvm/6.0.1' first when running CS225 assignments. $(ccend))
endif
else
ifneq ($(strip $(SKIP_EWS_CHECK)),True)
CLANG_VERSION_MSG = $(warning $(ccyellow) Looks like you are not on EWS. Be sure to test on EWS before the deadline. $(ccend))
endif
endif

.PHONY: all test clean output_msg

all : $(EXENAME)

output_msg: ; $(CLANG_VERSION_MSG)

$(EXENAME) : output_msg $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o $(EXENAME)

main.o : main.cpp Graph.h
	$(CXX) $(CXXFLAGS) main.cpp

Graph.o : Graph.cpp Graph.h
	$(CXX) $(CXXFLAGS) Graph.cpp

test : output_msg catchmain.o test.o Graph.o
	$(LD) catchmain.o test.o Graph.o $(LDFLAGS) -o test

catchmain.o : Catch/catchmain.cpp Catch/catch.hpp
	$(CXX) $(CXXFLAGS) Catch/catchmain.cpp

test.o : test.cpp Catch/catch.hpp Graph.cpp Graph.h
	$(CXX) $(CXXFLAGS) test.cpp

clean :
	-rm -f *.o $(EXENAME) test
