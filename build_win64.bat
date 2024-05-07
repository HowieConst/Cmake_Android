@echo off

rem 删除Build文件夹
set "BuildFolder=Build"
set "BuildDllFile=Release"
set "TargetDllPath=..\Output"

if exist "%BuildFolder%" (
    rd /s /q "%BuildFolder%"
)
mkdir "%BuildFolder%"
cd "%BuildFolder%"
cmake -G"Visual Studio 16 2019" -A x64 ..
cmake --build . --config Release

rem 拷贝dll
xcopy /Y "Release\PathFinding.dll" "%TargetDllPath%\PathFinding.dll"
pause
