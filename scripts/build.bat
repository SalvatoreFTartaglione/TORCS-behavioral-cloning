@echo off
rem Compiles all sources into .\out (run from anywhere).
cd /d "%~dp0\.."
if not exist out mkdir out
javac -encoding UTF-8 -d out src\scr\*.java
if errorlevel 1 exit /b 1
echo Build OK -^> out\
