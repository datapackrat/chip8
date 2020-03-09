if .%1.==.. goto whatfile
if exist %1 goto itexists
pcc %1 %2
if errorlevel 1 goto oops
pccl %1
del %1.o
if errorlevel 1 goto oops
goto exit
:whatfile
@echo Please specify a filename (without the .C extension)
goto exit
:itexists
@echo Please do not add the .C extension
goto exit
:oops
@echo Error in compile/link
:exit
