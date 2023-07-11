# Neighborhood Augmented Planning
# Makefile for compiling examples

SHELL := /bin/bash

# INCLUDE FOLDER
# --------------
DOSL_FOLDER = ..
INC_DOSL = -I../$(DOSL_FOLDER) # relative to the cpp sources

SGL_INC_FOLDER=..
INC_SGL=-I../$(SGL_INC_FOLDER)

INC_LOCAL = -I. -I..

# --------------------------------------------
# Macros
_BOLD = "\\033[1m"
_DIM = "\\033[2m"
_RED = "\\033[31m"
_GREEN = "\\033[32m"
_YELLOW = "\\033[33m"
_BLUE ="\\033[34m"
_GRAY = "\\033[37m"
_CLEAR = "\\033[0m"
# --------------------------------------------
# DOSL-specific
DOSL_ALGORITHM_LIST = $(shell find $(DOSL_FOLDER)/dosl/planners -type f -name '*.tcc' -exec basename {} .tcc \; | sort)

DOSL_DEFAULT_ALGORITHM = SStar

DOSL_ALGORITHM_X = $(shell \
                        if [ -z $(DOSL_ALGORITHM) ] ; then \
                            printf "$(_YELLOW)$(_BOLD)Enter algorithm name (AStar or SStar). Default algorithm is$(_CLEAR)$(_CLEAR) $(_YELLOW)$(DOSL_DEFAULT_ALGORITHM)$(_CLEAR)" >$$(tty); \
                            read -p ": " REPLY ; \
                            if [ -z $$REPLY ] ; then \
                                echo $(DOSL_DEFAULT_ALGORITHM) ; \
                            else \
                                echo $$REPLY ; \
                            fi \
                        else \
                            echo $(DOSL_ALGORITHM) ; \
                        fi )

DOSL_INTRO_STRING_DIR = "Including DOSL from '$(_YELLOW)$(DOSL_FOLDER)/$(_CLEAR)'.\n"
DOSL_INTRO_STRING_ALG = "$(eval DOSL_ALGORITHM ?= $(DOSL_ALGORITHM_X))Using algorithm '$(_YELLOW)$(DOSL_ALGORITHM)$(_CLEAR)' ($(_GRAY)$(_DIM)to change algorithm use 'make <target> DOSL_ALGORITHM=<algorithm>'$(_CLEAR)$(_CLEAR)).\n"


COMPILATION_SUCCESS_TEXT = "$(_GREEN)%s compilation successful!$(_CLEAR)\n"
COMPILATION_FAILURE_TEXT = "$(_RED)$(_BOLD)ERROR:$(_CLEAR) $(_RED)%s compilation failed!$(_CLEAR)\n"

COMPILE_PROGRAM = printf "$(_DIM)$(subst `,\`,$(strip $(2)))$(_CLEAR)\n" ; if $(2) ; then printf $(COMPILATION_SUCCESS_TEXT) "$(strip $(1))" ; else printf $(COMPILATION_FAILURE_TEXT) "$(strip $(1))" ; fi

# --------------------------------------------
# common flags

CC = g++
CFLAGS = -std=gnu++17 -g -O3
PROFILE = -pg
WARNS = -w

LIBS = -lm -pthread
LIBS_OPENCV = `pkg-config --cflags --libs opencv` -L/usr/local/share/OpenCV/3rdparty/lib/  # The -L option fixes bug with libippicv
LIBS_OPENGL := `pkg-config --cflags --libs x11 xi gl glu` -lglut

LIB_FOLDERS = -L/usr/local/lib

BIN_FOLDER = bin
SRC_FOLDER = src
2D_HEADERS = include/tools.cpp include/myNode2D.cpp
3D_HEADERS = include/tools.cpp include/myNode3D.cpp

# --------------------------------------------

.PHONY: 2D_NAG_Search
2D_NAG_Search:
	@printf "\nNow compiling '$(_BLUE)$@$(_CLEAR)'...\n"; printf $(DOSL_INTRO_STRING_DIR); printf $(DOSL_INTRO_STRING_ALG);
	@$(call COMPILE_PROGRAM,\
		$@,\
		$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_SGL) $(INC_DOSL) -o $(BIN_FOLDER)/$@_$(DOSL_ALGORITHM) $(SRC_FOLDER)/$@.cpp $(2D_HEADERS) $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV) $(LIBS_OPENGL) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)\
	)

.PHONY: 3D_NAG_Search
3D_NAG_Search:
	@printf "\nNow compiling '$(_BLUE)$@$(_CLEAR)'...\n"; printf $(DOSL_INTRO_STRING_DIR); printf $(DOSL_INTRO_STRING_ALG);
	@$(call COMPILE_PROGRAM,\
		$@,\
		$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_SGL) $(INC_DOSL) -o $(BIN_FOLDER)/$@_$(DOSL_ALGORITHM) $(SRC_FOLDER)/$@.cpp $(3D_HEADERS) $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV) $(LIBS_OPENGL) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)\
	)

clean:
	rm bin/*


