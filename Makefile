##########################  Macro Definitions  ############################

# Let's try to auto-detect what platform we're on.
# If this fails, set 42PLATFORM manually in the else block.
AUTOPLATFORM = Failed
ifeq ($(MSYSTEM),MINGW32)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __MSYS__
endif
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __linux__
endif
ifeq ($(UNAME_S),Darwin)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __APPLE__
endif
ifeq ($(AUTOPLATFORM),Failed)
   # Autodetect failed.  Set platform manually.
   #42PLATFORM = __APPLE__
   #42PLATFORM = __linux__
   42PLATFORM = __MSYS__
endif


GUIFLAG = -D _ENABLE_GUI_
#GUIFLAG =

SHADERFLAG = -D _USE_SHADERS_
#SHADERFLAG =

CFDFLAG =
#CFDFLAG = -D _ENABLE_CFD_SLOSH_

FFTBFLAG =
#FFTBFLAG = -D _ENABLE_FFTB_CODE_

GSFCFLAG =
#GSFCFLAG = -D _USE_GSFC_WATERMARK_

STANDALONEFLAG =
#STANDALONEFLAG = -D _AC_STANDALONE_

GMSECFLAG =
#GMSECFLAG = -D _ENABLE_GMSEC_

ifeq ($(strip $(GMSECFLAG)),)
   GMSECDIR =
   GMSECINC =
   GMSECBIN =
   GMSECLIB =
else
   GMSECDIR = ~/GMSEC/
   GMSECINC = -I $(GMSECDIR)include/
   GMSECBIN = -L $(GMSECDIR)bin/
   GMSECLIB = -lGMSECAPI
endif

# Basic directories
HOMEDIR = ./
PROJDIR = ./
KITDIR = $(PROJDIR)Kit/
OBJ = $(PROJDIR)Object/
INC = $(PROJDIR)Include/
SRC = $(PROJDIR)Source/
KITINC = $(KITDIR)Include/
KITSRC = $(KITDIR)Source/
INOUT = $(PROJDIR)InOut/
GSFCSRC = $(PROJDIR)../GSFC/
AUTOSRC = $(SRC)AutoCode/
META = $(PROJDIR)MetaCode/


ifeq ($(42PLATFORM),__APPLE__)
   # Mac Macros
   CINC = -I /usr/include -I /usr/local/include
   EXTERNDIR =
   # ARCHFLAG = -arch i386
   # ARCHFLAG = -arch x86_64
   # ARCHFLAG = -arch arm64
   ARCHFLAG = -arch $(shell uname -m)
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   # OSX fixed their hires GLUT issue.  Keep GLFW around just in case.
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   LFLAGS = 
   ifneq ($(strip $(GUIFLAG)),)
      GLINC = -I /System/Library/Frameworks/OpenGL.framework/Headers/ -I /System/Library/Frameworks/GLUT.framework/Headers/
      ifeq ($(strip $(GLUT_OR_GLFW)),_USE_GLUT_)
         LIBS = -framework System -framework Carbon -framework OpenGL -framework GLUT
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         GUI_LIB = -D _USE_GLUT_
      else
         LIBS = -lglfw -framework System -framework Carbon -framework OpenGL -framework GLUT
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glfw.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         GUI_LIB = -D _USE_GLFW_
      endif
   else
      GLINC = 
      LIBS = 
      GUIOBJ = 
   endif
   NOS3OBJ = $(OBJ)42nos3.o
   XWARN = 
   EXENAME = 42
   CC = gcc
endif

ifeq ($(42PLATFORM),__linux__)
   # Linux Macros
   CINC =
   EXTERNDIR =
   ARCHFLAG =
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   ifneq ($(strip $(GUIFLAG)),)
      ifeq ($(strip $(GLUT_OR_GLFW)),_USE_GLUT_)
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         LIBS = -lglut -lGLU -lGL -ldl -lm -lpthread
         GLINC = -I /usr/include/GL/
         LFLAGS = -L $(KITDIR)/GL/lib/
         GUI_LIB = -D _USE_GLUT_
      else
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glfw.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         LIBS = -lglfw -lglut -lGLU -lGL -ldl -lm -lpthread
         GLINC = -I /usr/include/GL/ -I /usr/include/GLFW
         GUI_LIB = -D _USE_GLFW_
      endif
   else
      GUIOBJ =
      GLINC =
      LIBS = -ldl -lm -lpthread
      LFLAGS =
   endif
   NOS3OBJ = $(OBJ)42nos3.o 
   XWARN = -Wno-unused-variable -Wno-unused-but-set-variable -Wno-stringop-overread
   EXENAME = 42
   CC = gcc
endif

ifeq ($(42PLATFORM),__MSYS__)
   CINC =
   EXTERNDIR = /c/42ExternalSupport/
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   ifneq ($(strip $(GUIFLAG)),)
      # TODO: Option to use GLFW instead of GLUT?
      GLEW = $(EXTERNDIR)GLEW/
      # GLEW = /mingw64/
      GLUT = $(EXTERNDIR)freeglut/
      LIBS =  -lopengl32 -lglu32 -lfreeglut -lws2_32 -lglew32
      LFLAGS = -L $(GLUT)lib/ -L $(GLEW)lib/
      GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
      GLINC = -I $(GLEW)include/GL/ -I $(GLUT)include/GL/
      ARCHFLAG = -D GLUT_NO_LIB_PRAGMA -D GLUT_NO_WARNING_DISABLE -D GLUT_DISABLE_ATEXIT_HACK
   else
      GUIOBJ =
      GLINC =
      LIBS =  -lws2_32
      LFLAGS =
      ARCHFLAG =
   endif
   NOS3OBJ = 
   XWARN = 
   EXENAME = 42.exe
   CC = gcc
endif

# If not using GUI, don't compile GUI-related files
ifeq ($(strip $(GUIFLAG)),)
   GUIOBJ =
endif

# If not in FFTB, don't compile FFTB-related files
ifneq ($(strip $(FFTBFLAG)),)
   FFTBOBJ = $(OBJ)42fftb.o
else
   FFTBOBJ =
endif

ifneq ($(strip $(CFDFLAG)),)
   SLOSHOBJ = $(OBJ)42CfdSlosh.o
else
   SLOSHOBJ =
endif

# If not _AC_STANDALONE_, link AcApp.c in with the rest of 42
ifneq ($(strip $(STANDALONEFLAG)),)
   ACOBJ =
   ACIPCOBJ = $(OBJ)AcIPC.o
   SCIPCOBJ = $(OBJ)ScIPC.o
else
   ACOBJ = $(OBJ)AcApp.o
   ACIPCOBJ = 
   SCIPCOBJ = 
endif

# TODO: Reconstitute if needed
#ifneq ($(strip $(GMSECFLAG)),)
#   GMSECOBJ = $(OBJ)gmseckit.o
#   ACIPCOBJ = 
#   SCIPCOBJ = 
#else
#   GMSECOBJ =
#   ACIPCOBJ = 
#   SCIPCOBJ = 
#endif

42OBJ = $(OBJ)42main.o $(OBJ)42exec.o $(OBJ)42actuators.o $(OBJ)42cmd.o \
$(OBJ)42dynamics.o $(OBJ)42environs.o $(OBJ)42ephem.o $(OBJ)42fsw.o \
$(OBJ)42init.o $(OBJ)42ipc.o $(OBJ)42jitter.o $(OBJ)42joints.o \
$(OBJ)42optics.o $(OBJ)42perturb.o $(OBJ)42report.o $(OBJ)42sensors.o

KITOBJ = $(OBJ)dcmkit.o $(OBJ)envkit.o $(OBJ)fswkit.o  $(OBJ)iokit.o \
$(OBJ)mathkit.o $(OBJ)meshkit.o $(OBJ)nrlmsise00kit.o $(OBJ)orbkit.o \
$(OBJ)radbeltkit.o $(OBJ)sigkit.o $(OBJ)sphkit.o $(OBJ)timekit.o

LIBKITOBJ = $(OBJ)dcmkit.o $(OBJ)envkit.o $(OBJ)fswkit.o $(OBJ)iokit.o \
$(OBJ)mathkit.o $(OBJ)meshkit.o $(OBJ)orbkit.o $(OBJ)sigkit.o $(OBJ)sphkit.o \
$(OBJ)timekit.o

ACKITOBJ = $(OBJ)dcmkit.o $(OBJ)mathkit.o $(OBJ)fswkit.o $(OBJ)iokit.o $(OBJ)timekit.o

AUTOOBJ = $(OBJ)WriteAcToCsv.o $(OBJ)WriteScToCsv.o $(OBJ)TxRxIPC.o

#ANSIFLAGS = -Wstrict-prototypes -pedantic -ansi -Werror
ANSIFLAGS =

CFLAGS = -std=c11 -g -O0 -fpic -Wall -Wshadow -Wno-deprecated $(XWARN) $(ANSIFLAGS) $(GLINC) $(CINC) -I $(INC) -I $(KITINC) -I $(KITSRC) $(GMSECINC) $(ARCHFLAG) $(GUIFLAG) $(GUI_LIB) $(SHADERFLAG) $(CFDFLAG) $(FFTBFLAG) $(GSFCFLAG) $(GMSECFLAG) $(STANDALONEFLAG)


##########################  Rules to link 42  #############################

42 : $(42OBJ) $(KITOBJ) $(GUIOBJ) $(NOS3OBJ) $(AUTOOBJ) $(ACOBJ) $(SCIPCOBJ) $(GMSECOBJ) $(FFTBOBJ) $(SLOSHOBJ)
	$(CC) $(LFLAGS) $(GMSECBIN) -o $(EXENAME) $(42OBJ) $(KITOBJ) $(GUIOBJ) $(NOS3OBJ) $(AUTOOBJ) $(ACOBJ) $(SCIPCOBJ) $(GMSECOBJ) $(FFTBOBJ) $(SLOSHOBJ) $(LIBS)

AcApp : $(OBJ)AcApp.o $(ACKITOBJ) $(ACIPCOBJ) $(GMSECOBJ)
	$(CC) $(LFLAGS) -o AcApp $(OBJ)AcApp.o $(ACKITOBJ) $(ACIPCOBJ) $(GMSECOBJ) $(LIBS)
	
42kit : $(LIBKITOBJ)
	$(CC) $(LFLAGS) -shared -o $(KITDIR)42kit.so $(LIBKITOBJ)


####################  Rules to compile objects  ###########################

$(OBJ)42main.o      : $(SRC)42main.c
	$(CC) $(CFLAGS) -c $(SRC)42main.c -o $(OBJ)42main.o

$(OBJ)42exec.o      : $(SRC)42exec.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42exec.c -o $(OBJ)42exec.o

$(OBJ)42actuators.o : $(SRC)42actuators.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42actuators.c -o $(OBJ)42actuators.o

$(OBJ)42cmd.o : $(SRC)42cmd.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42cmd.c -o $(OBJ)42cmd.o

$(OBJ)42dynamics.o  : $(SRC)42dynamics.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42dynamics.c -o $(OBJ)42dynamics.o

$(OBJ)42environs.o  : $(SRC)42environs.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42environs.c -o $(OBJ)42environs.o

$(OBJ)42ephem.o     : $(SRC)42ephem.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42ephem.c -o $(OBJ)42ephem.o

$(OBJ)42fsw.o       : $(SRC)42fsw.c $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42fsw.c -o $(OBJ)42fsw.o

$(OBJ)42gl.o        : $(SRC)42gl.c $(INC)42.h $(INC)42gl.h
	$(CC) $(CFLAGS) -c $(SRC)42gl.c -o $(OBJ)42gl.o

$(OBJ)42glfw.o	: $(SRC)42glfw.c $(INC)42.h $(INC)42gl.h $(INC)42glfw.h
	$(CC) $(CFLAGS) -c $(SRC)42glfw.c -o $(OBJ)42glfw.o

$(OBJ)42glut.o      : $(SRC)42glut.c $(INC)42.h $(INC)42gl.h $(INC)42glut.h
	$(CC) $(CFLAGS) -c $(SRC)42glut.c -o $(OBJ)42glut.o

$(OBJ)42gpgpu.o      : $(SRC)42gpgpu.c $(INC)42.h $(INC)42gl.h $(INC)42glut.h
	$(CC) $(CFLAGS) -c $(SRC)42gpgpu.c -o $(OBJ)42gpgpu.o

$(OBJ)42init.o      : $(SRC)42init.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42init.c -o $(OBJ)42init.o

$(OBJ)42ipc.o       : $(SRC)42ipc.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42ipc.c -o $(OBJ)42ipc.o

$(OBJ)42jitter.o    : $(SRC)42jitter.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42jitter.c -o $(OBJ)42jitter.o

$(OBJ)42joints.o    : $(SRC)42joints.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42joints.c -o $(OBJ)42joints.o

$(OBJ)42optics.o   : $(SRC)42optics.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42optics.c -o $(OBJ)42optics.o

$(OBJ)42perturb.o   : $(SRC)42perturb.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42perturb.c -o $(OBJ)42perturb.o

$(OBJ)42report.o    : $(SRC)42report.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42report.c -o $(OBJ)42report.o

$(OBJ)42sensors.o   : $(SRC)42sensors.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42sensors.c -o $(OBJ)42sensors.o

$(OBJ)dcmkit.o      : $(KITSRC)dcmkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)dcmkit.c -o $(OBJ)dcmkit.o

$(OBJ)envkit.o      : $(KITSRC)envkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)envkit.c -o $(OBJ)envkit.o

$(OBJ)fswkit.o      : $(KITSRC)fswkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)fswkit.c -o $(OBJ)fswkit.o

$(OBJ)glkit.o      : $(KITSRC)glkit.c $(KITINC)glkit.h
	$(CC) $(CFLAGS) -c $(KITSRC)glkit.c -o $(OBJ)glkit.o

$(OBJ)gmseckit.o      : $(KITSRC)gmseckit.c $(KITINC)gmseckit.h
	$(CC) $(CFLAGS) -c $(KITSRC)gmseckit.c -o $(OBJ)gmseckit.o

$(OBJ)iokit.o      : $(KITSRC)iokit.c
	$(CC) $(CFLAGS) -c $(KITSRC)iokit.c -o $(OBJ)iokit.o

$(OBJ)mathkit.o     : $(KITSRC)mathkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)mathkit.c -o $(OBJ)mathkit.o

$(OBJ)meshkit.o      : $(KITSRC)meshkit.c $(KITINC)meshkit.h
	$(CC) $(CFLAGS) -c $(KITSRC)meshkit.c -o $(OBJ)meshkit.o

$(OBJ)nrlmsise00kit.o   : $(KITSRC)nrlmsise00kit.c
	$(CC) $(CFLAGS) -c $(KITSRC)nrlmsise00kit.c -o $(OBJ)nrlmsise00kit.o

$(OBJ)msis86kit.o   : $(KITSRC)msis86kit.c $(KITINC)msis86kit.h
	$(CC) $(CFLAGS) -c $(KITSRC)msis86kit.c -o $(OBJ)msis86kit.o

$(OBJ)orbkit.o      : $(KITSRC)orbkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)orbkit.c -o $(OBJ)orbkit.o

$(OBJ)radbeltkit.o      : $(KITSRC)radbeltkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)radbeltkit.c -o $(OBJ)radbeltkit.o

$(OBJ)sigkit.o      : $(KITSRC)sigkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)sigkit.c -o $(OBJ)sigkit.o

$(OBJ)sphkit.o      : $(KITSRC)sphkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)sphkit.c -o $(OBJ)sphkit.o

$(OBJ)timekit.o     : $(KITSRC)timekit.c
	$(CC) $(CFLAGS) -c $(KITSRC)timekit.c -o $(OBJ)timekit.o

$(OBJ)42CfdSlosh.o      : $(GSFCSRC)42CfdSlosh.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(GSFCSRC)42CfdSlosh.c -o $(OBJ)42CfdSlosh.o

$(OBJ)42fftb.o         : $(GSFCSRC)42fftb.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(GSFCSRC)42fftb.c -o $(OBJ)42fftb.o

$(OBJ)AcApp.o          : $(SRC)AcApp.c $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)AcApp.c -o $(OBJ)AcApp.o

$(OBJ)WriteAcToCsv.o  : $(AUTOSRC)WriteAcToCsv.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(AUTOSRC)WriteAcToCsv.c -o $(OBJ)WriteAcToCsv.o

$(OBJ)WriteScToCsv.o  : $(AUTOSRC)WriteScToCsv.c $(INC)42.h $(INC)42types.h
	$(CC) $(CFLAGS) -c $(AUTOSRC)WriteScToCsv.c -o $(OBJ)WriteScToCsv.o
	
$(OBJ)TxRxIPC.o  : $(AUTOSRC)TxRxIPC.c $(INC)42.h $(INC)42types.h
	$(CC) $(CFLAGS) -c $(AUTOSRC)TxRxIPC.c -o $(OBJ)TxRxIPC.o
	
$(OBJ)ScIPC.o  : $(AUTOSRC)ScIPC.c $(INC)42.h $(INC)42types.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(AUTOSRC)ScIPC.c -o $(OBJ)ScIPC.o
	
$(OBJ)AcIPC.o  : $(AUTOSRC)AcIPC.c $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(AUTOSRC)AcIPC.c -o $(OBJ)AcIPC.o
	
#$(AUTOSRC)AcIPC.c  : $(META)Ac.json
#	julia $(META)JsonToAcIPC.jl
	
#$(AUTOSRC)ScIPC.c  : $(META)Ac.json
#	julia $(META)JsonToAcIPC.jl
	
#$(AUTOSRC)TxRxIPC.c  : $(META)42.json $(META)Ac.json
#	julia $(META)JsonToTxRxIPC.jl
	
#$(AUTOSRC)WriteAcToCsv.c  : $(META)Ac.json
#	julia $(META)JsonToAcCsv.jl

#$(AUTOSRC)WriteScToCsv.c  : $(META)42.json
#	julia $(META)JsonToScCsv.jl

#$(META)42.json  : $(INC)42types.h $(KITINC)orbkit.h
#	julia $(META)HeadersToJson.jl

#$(META)Ac.json  : $(INC)AcTypes.h 
#	julia $(META)HeadersToJson.jl

$(OBJ)42nos3.o         : $(SRC)42nos3.c
	$(CC) $(CFLAGS) -c $(SRC)42nos3.c -o $(OBJ)42nos3.o


########################  Miscellaneous Rules  ############################
clean :
ifeq ($(42PLATFORM),_WIN32)
	del .\Object\*.o .\$(EXENAME) .\InOut\*.42
else ifeq ($(42PLATFORM),_WIN64)
	del .\Object\*.o .\$(EXENAME) .\InOut\*.42
else
	rm -f $(OBJ)*.o ./$(EXENAME) ./AcApp $(KITDIR)42kit.so 
	rm -f $(INOUT)*.42 ./Standalone/*.42 ./Demo/*.42 ./Rx/*.42 ./Tx/*.42 ./Rover/*.42
	rm -f $(INOUT)*.csv ./Standalone/*.csv ./Demo/*.csv ./Rx/*.csv ./Tx/*.csv ./Rover/*.csv
endif
