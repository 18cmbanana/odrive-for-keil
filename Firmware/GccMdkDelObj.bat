chdir %~dp0
echo Congratulations! Work is doing.

del *.o /s
del *.d /s

del .\*.o /s
del .\*.d /s

del .\*.crf /s

del .\*.lst /s

del .\*.dep /s

del .\*.orig /s

exit
