@echo off
rem Usage: scripts\run.bat [drive^|train]
rem Extra JVM options can be passed through JAVA_OPTS, e.g. set JAVA_OPTS=-Dtorcs.data=data\other.csv
cd /d "%~dp0\.."
set MODE=%1
if "%MODE%"=="" set MODE=drive
if not exist out call scripts\build.bat
java -Dfile.encoding=UTF-8 -Dtorcs.mode=%MODE% %JAVA_OPTS% -cp out scr.Client scr.SimpleDriver host:localhost port:3001
