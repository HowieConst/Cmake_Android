@echo off

set ANDROID_NDK=%~dp0android-ndk-r16b

echo %ANDROID_NDK%
rem 删除build文件夹
set "buildfolder=buildandroid"
set "builddllfile=release"
set "targetdllpath=..\output"

if exist "%buildfolder%" (
  rd /s /q "%buildfolder%"
)
mkdir "%buildfolder%"
cd "%buildfolder%"
cmake -DANDROID_NDK=%android_ndk% ^
	  -DANDROID_TOOLCHAIN=../cmake/android.toolchain.cmake ^
      -DANDROID_ABI=armeabi-v7a ^
      -DANDROID_NATIVE_API_LEVEL=android-19 ^
      -DANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang3.4 ^
	  ..
cmake --build ..
 
rem cmake -S . -B %buildfolder% -DCMAKE_TOOLCHAIN_FILE=cmake/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DANDROID_NDK=%ANDROID_NDK%
pause