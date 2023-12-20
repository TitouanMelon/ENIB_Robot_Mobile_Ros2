doxygen ./robotROS
./latex/make.bat

exit

"c:\Program Files\WinRAR\WinRAR.exe" a -afzip ./html_doc.zip ./html
"c:\Program Files\WinRAR\WinRAR.exe" a -afzip ./pdf_doc.zip ./latex/refman.pdf ./externe/base_robot.pdf ./externe/vl53l0x-datasheet-I2C.pdf