@ECHO OFF

echo "Generating UML..."
pyreverse -o png -p "AM3 Simulator" ./src/am3/
if not exist ".\build\uml\" mkdir .\build\uml

copy .\classes_AM3_Simulator.png .\build\uml\ /y
copy .\classes_AM3_Simulator.png .\src\_static\img\ /y
del .\classes_AM3_Simulator.png
del .\packages_AM3_Simulator.png

pushd %~dp0

REM Command file for Sphinx documentation

if "%SPHINXBUILD%" == "" (
	set SPHINXBUILD=sphinx-build
)
set SOURCEDIR=src
set BUILDDIR=build

if "%1" == "" goto help

%SPHINXBUILD% >NUL 2>NUL
if errorlevel 9009 (
	echo.
	echo.The 'sphinx-build' command was not found. Make sure you have Sphinx
	echo.installed, then set the SPHINXBUILD environment variable to point
	echo.to the full path of the 'sphinx-build' executable. Alternatively you
	echo.may add the Sphinx directory to PATH.
	echo.
	echo.If you don't have Sphinx installed, grab it from
	echo.http://sphinx-doc.org/
	exit /b 1
)

%SPHINXBUILD% -M %1 %SOURCEDIR% %BUILDDIR% %SPHINXOPTS%
goto end

:help
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS%

:end
popd
