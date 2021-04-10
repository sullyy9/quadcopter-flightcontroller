@ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION 
SET "build_dir=%~dp0%..\build"
SET "root_dir=%~dp0%.."

SET "build_commands_file=%build_dir%\compile_commands.json"
SET "build_temp_file=%build_dir%\temp.json"

SET "root_commands_file=%root_dir%\compile_commands.json"
SET "root_temp_file=%root_dir%\temp.json"

:: Clean the build directory
rmdir /s /q %build_dir%
mkdir %build_dir%

:: Create the cmake configuration
cmake -S%root_dir% -B%build_dir% -G"MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=%root_dir%\toolchain.cmake

:: Fix the compile_commands file, proccess it, then fix it again
(for /f "delims=" %%a in (%build_commands_file%) do (
    SET s=%%a
    SET s=!s:\\=/!
    echo !s!
)) >%build_temp_file%

move /Y %build_temp_file% %build_commands_file%

compdb -p %build_dir% list > compile_commands.json

(for /f "delims=" %%a in (%root_commands_file%) do (
    SET s=%%a
    SET s=!s:\\\\=/!
    SET s=!s:\\=/!
    echo !s!
)) >%root_temp_file%

move /Y %root_temp_file% %root_commands_file%