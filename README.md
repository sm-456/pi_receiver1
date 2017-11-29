# Raspberry Pi Receiver for Sensor Network

communication with SPIRIT1 transceiver via SPI
Raspberry Pi uses wiringPi

Raspberry Pi will be in receiver mode most of the time
for communication protocol see documentation

```c

build all:
gcc -Wall -o "main" "main.c" SPIRIT_Commands.c wiringPiSPI.c buffer.c globals.c Register_Setting.c SPIRIT_Aes.c SPIRIT_Calibration.c SPIRIT_Csma.c SPIRIT_DirectRF.c SPIRIT_General.c SPIRIT_Gpio.c SPIRIT_Irq.c SPIRIT_LinearFifo.c SPIRIT_Management.c SPIRIT_PktBasic.c SPIRIT_PktCommon.c SPIRIT_PktMbus.c SPIRIT_PktStack.c SPIRIT_Qi.c SPIRIT_Radio.c SPIRIT_Timer.c SPIRIT_Types.c -l wiringPi

GIT
git status

datei hinzufuegen
git add datei.x
git add --all

commit nach änderungen:
git commit -am 'kommentar' 

neuer branch
git checkout -b newbranch

branch info
git branch

branch wechseln
git checkout otherbranch

merge (in master):
git merge otherbranch

GitHub push:
git push origin branch

Datei Historie:
git log datei.x
git log -p datei.x

remote origin hinzufuegen:
git remote add name adresse

remote origin anzeigen:
git remote -v
```
~sm
