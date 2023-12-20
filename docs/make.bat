del .\html_doc.zip
del .\pdf_doc.zip
doxygen ./robotROS_doxygen_conf
"c:\Program Files\WinRAR\WinRAR.exe" a -afzip ./html_doc.zip ./html
echo "c:\Program Files\WinRAR\WinRAR.exe" a -afzip -ep ./pdf_doc.zip ./latex/refman.pdf ./externe/base_robot.pdf ./externe/vl53l0x-datasheet-I2C.pdf >> ./latex/make.bat
./latex/make.bat