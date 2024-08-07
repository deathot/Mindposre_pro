set YMD=%date:~0,4%%date:~5,2%%date:~8,2%
if exist %YMD%_output (
echo "Delete %YMD%_output dir!"
rd /S /Q %YMD%_output
) 
md %YMD%_output
.\build > .\%YMD%_output\%YMD%_log.log

