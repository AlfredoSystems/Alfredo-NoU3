@echo off
REM Double-click launcher for the NoU3 MotorParty auto-flasher.
REM Keeps the window open after it exits so you can read the summary.
cd /d "%~dp0"
python flash_motorparty.py %*
echo.
pause
