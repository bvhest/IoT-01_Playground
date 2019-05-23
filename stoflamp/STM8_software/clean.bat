rem clean-up stm8 stuff
rem
cd Button
rmdir /S /Q Debug
del Release\*.bak Release\*.elf Release\*.lkf Release\*.ls Release\*.o Release\*.bak Release\*.map Release\*.sm8
cd ..
cd Lamp
rmdir /S /Q Debug
del Release\*.bak Release\*.elf Release\*.lkf Release\*.ls Release\*.o Release\*.bak Release\*.map Release\*.sm8
cd ..
