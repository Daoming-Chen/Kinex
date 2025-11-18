# setup.ps1 - Setup script for urdfx development environment on Windows
# This script checks for required dependencies and provides installation instructions
# Usage: .\scripts\setup.ps1

#Requires -Version 5.1

param(
    [switch]$SkipEmscripten = $false
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$EmsdkDir = Join-Path $ProjectRoot "third_party\emsdk"

# Colors for output
function Write-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Green
}

function Write-Warn {
    param([string]$Message)
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Error-Custom {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

# Detect Windows version
function Get-WindowsVersion {
    $osInfo = Get-CimInstance -ClassName Win32_OperatingSystem
    $version = $osInfo.Version
    $caption = $osInfo.Caption
    
    Write-Info "Detected OS: $caption (Version $version)"
    
    # Check if Windows 10 1809+ or Windows 11
    $versionParts = $version.Split('.')
    $major = [int]$versionParts[0]
    $build = [int]$versionParts[2]
    
    if ($major -lt 10) {
        Write-Error-Custom "Windows 10 (version 1809+) or Windows 11 is required"
        return $false
    }
    
    if ($major -eq 10 -and $build -lt 17763) {
        Write-Warn "Windows 10 build $build detected. Build 17763+ (version 1809) is recommended"
    }
    
    return $true
}

# Check CMake installation
function Test-CMake {
    Write-Info "Checking for CMake..."
    
    if (Get-Command cmake -ErrorAction SilentlyContinue) {
        $cmakeVersion = (cmake --version | Select-Object -First 1) -replace 'cmake version ', ''
        Write-Info "CMake found: version $cmakeVersion"
        
        # Check version >= 3.20
        $versionParts = $cmakeVersion.Split('.')
        $major = [int]$versionParts[0]
        $minor = [int]$versionParts[1]
        
        if ($major -gt 3 -or ($major -eq 3 -and $minor -ge 20)) {
            return $true
        } else {
            Write-Warn "CMake version $cmakeVersion is too old (need >= 3.20)"
            return $false
        }
    }
    
    return $false
}

function Show-CMakeInstallInstructions {
    Write-Info "CMake 3.20+ is required but not found"
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor Cyan
    Write-Host "  1. Using Chocolatey (recommended):" -ForegroundColor White
    Write-Host "     choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  2. Manual download:" -ForegroundColor White
    Write-Host "     Visit: https://cmake.org/download/" -ForegroundColor Yellow
    Write-Host "     Download and run the installer, select 'Add CMake to PATH'" -ForegroundColor Yellow
    Write-Host ""
}

# Check Python installation
function Test-Python {
    Write-Info "Checking for Python..."
    
    if (Get-Command python -ErrorAction SilentlyContinue) {
        $pythonVersion = python --version 2>&1
        Write-Info "Python found: $pythonVersion"
        
        # Extract version number
        if ($pythonVersion -match 'Python (\d+)\.(\d+)') {
            $major = [int]$Matches[1]
            $minor = [int]$Matches[2]
            
            if ($major -gt 3 -or ($major -eq 3 -and $minor -ge 8)) {
                return $true
            } else {
                Write-Warn "Python version is too old (need >= 3.8)"
                return $false
            }
        }
    }
    
    return $false
}

function Show-PythonInstallInstructions {
    Write-Info "Python 3.8+ is required but not found"
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor Cyan
    Write-Host "  1. Using Chocolatey (recommended):" -ForegroundColor White
    Write-Host "     choco install python --version=3.11.0" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  2. Manual download:" -ForegroundColor White
    Write-Host "     Visit: https://www.python.org/downloads/" -ForegroundColor Yellow
    Write-Host "     Download Python 3.8+ installer" -ForegroundColor Yellow
    Write-Host "     During installation, check 'Add Python to PATH'" -ForegroundColor Yellow
    Write-Host ""
}

# Check Node.js installation
function Test-NodeJS {
    Write-Info "Checking for Node.js..."
    
    if (Get-Command node -ErrorAction SilentlyContinue) {
        $nodeVersion = node --version
        Write-Info "Node.js found: version $nodeVersion"
        return $true
    }
    
    return $false
}

function Show-NodeJSInstallInstructions {
    Write-Info "Node.js is required for visualization app but not found"
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor Cyan
    Write-Host "  1. Using Chocolatey (recommended):" -ForegroundColor White
    Write-Host "     choco install nodejs-lts" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  2. Manual download:" -ForegroundColor White
    Write-Host "     Visit: https://nodejs.org/" -ForegroundColor Yellow
    Write-Host "     Download the LTS version installer" -ForegroundColor Yellow
    Write-Host ""
}

# Check Visual Studio/MSVC installation
function Test-VisualStudio {
    Write-Info "Checking for Visual Studio/MSVC..."
    
    # Check for vswhere tool (comes with VS 2017+)
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    
    if (Test-Path $vswhere) {
        $vsInstances = & $vswhere -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -format json | ConvertFrom-Json
        
        if ($vsInstances) {
            foreach ($instance in $vsInstances) {
                $displayName = $instance.displayName
                $version = $instance.installationVersion
                Write-Info "Found: $displayName (Version $version)"
                
                # Check if version >= 16.11 (MSVC 19.29)
                $versionParts = $version.Split('.')
                $major = [int]$versionParts[0]
                $minor = [int]$versionParts[1]
                
                if ($major -gt 16 -or ($major -eq 16 -and $minor -ge 11)) {
                    Write-Info "Visual Studio meets minimum requirements (16.11+)"
                    return $true
                } else {
                    Write-Warn "Visual Studio version is too old. Need 16.11+ (VS 2019 16.11)"
                }
            }
        }
    }
    
    # Try to detect MSVC through cl.exe
    if (Get-Command cl -ErrorAction SilentlyContinue) {
        Write-Info "MSVC compiler (cl.exe) found in PATH"
        # Version check would require parsing cl output
        return $true
    }
    
    return $false
}

function Show-VisualStudioInstallInstructions {
    Write-Info "Visual Studio 2019 (16.11+) or Visual Studio 2022 is required"
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor Cyan
    Write-Host "  1. Visual Studio Community (free for individuals and open source):" -ForegroundColor White
    Write-Host "     Visit: https://visualstudio.microsoft.com/downloads/" -ForegroundColor Yellow
    Write-Host "     Install 'Desktop development with C++' workload" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  2. Visual Studio Build Tools (command-line only):" -ForegroundColor White
    Write-Host "     Visit: https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022" -ForegroundColor Yellow
    Write-Host "     Install 'C++ build tools' workload" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  3. Using Chocolatey:" -ForegroundColor White
    Write-Host "     choco install visualstudio2022community --package-parameters '--add Microsoft.VisualStudio.Workload.NativeDesktop'" -ForegroundColor Yellow
    Write-Host ""
}

# Setup Emscripten SDK
function Install-Emscripten {
    if ($SkipEmscripten) {
        Write-Info "Skipping Emscripten setup (--SkipEmscripten flag provided)"
        return
    }
    
    Write-Info "Checking Emscripten SDK..."
    
    if (Test-Path (Join-Path $EmsdkDir "emsdk.bat")) {
        Write-Info "Emscripten SDK directory already exists at $EmsdkDir"
        
        # Check if configured
        if (Test-Path (Join-Path $EmsdkDir ".emscripten")) {
            Write-Info "Emscripten SDK appears to be configured"
            return
        }
    }
    
    Write-Info "Setting up Emscripten SDK at $EmsdkDir..."
    
    # Create third_party directory if needed
    $thirdPartyDir = Split-Path -Parent $EmsdkDir
    if (-not (Test-Path $thirdPartyDir)) {
        New-Item -ItemType Directory -Path $thirdPartyDir | Out-Null
    }
    
    # Clone emsdk if not present
    if (-not (Test-Path $EmsdkDir)) {
        Write-Info "Cloning Emscripten SDK repository..."
        git clone https://github.com/emscripten-core/emsdk.git $EmsdkDir
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "Failed to clone Emscripten SDK"
            return
        }
    }
    
    # Install and activate latest emsdk
    Write-Info "Installing latest Emscripten..."
    Push-Location $EmsdkDir
    try {
        & .\emsdk.bat install latest
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "Failed to install Emscripten"
            return
        }
        
        & .\emsdk.bat activate latest
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "Failed to activate Emscripten"
            return
        }
        
        Write-Info "Emscripten SDK installed successfully"
        Write-Info "To use Emscripten, run: $EmsdkDir\emsdk_env.bat"
    } finally {
        Pop-Location
    }
}

# Main setup flow
function Main {
    Write-Host ""
    Write-Info "Starting urdfx development environment setup for Windows..."
    Write-Host ""
    
    # Check Windows version
    if (-not (Get-WindowsVersion)) {
        exit 1
    }
    Write-Host ""
    
    # Track missing dependencies
    $missingDeps = @()
    
    # Check CMake
    if (-not (Test-CMake)) {
        Show-CMakeInstallInstructions
        $missingDeps += "CMake"
    }
    Write-Host ""
    
    # Check Python
    if (-not (Test-Python)) {
        Show-PythonInstallInstructions
        $missingDeps += "Python"
    }
    Write-Host ""
    
    # Check Node.js
    if (-not (Test-NodeJS)) {
        Show-NodeJSInstallInstructions
        $missingDeps += "Node.js"
    }
    Write-Host ""
    
    # Check Visual Studio
    if (-not (Test-VisualStudio)) {
        Show-VisualStudioInstallInstructions
        $missingDeps += "Visual Studio/MSVC"
    }
    Write-Host ""
    
    # Setup Emscripten
    Install-Emscripten
    Write-Host ""
    
    # Summary
    if ($missingDeps.Count -gt 0) {
        Write-Warn "Setup incomplete. Missing dependencies: $($missingDeps -join ', ')"
        Write-Host ""
        Write-Host "Please install the missing dependencies and run this script again." -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Quick install with Chocolatey (run as Administrator):" -ForegroundColor Cyan
        Write-Host "  Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))" -ForegroundColor Yellow
        Write-Host "  choco install cmake python nodejs-lts visualstudio2022community --package-parameters '--add Microsoft.VisualStudio.Workload.NativeDesktop'" -ForegroundColor Yellow
        Write-Host ""
        exit 1
    }
    
    Write-Info "Setup complete! All required dependencies are installed."
    Write-Host ""
    Write-Info "Next steps:"
    Write-Host "  1. Initialize git submodules:" -ForegroundColor White
    Write-Host "     git submodule update --init --recursive" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  2. Configure and build the project:" -ForegroundColor White
    Write-Host "     cmake -B build -DCMAKE_BUILD_TYPE=Release" -ForegroundColor Yellow
    Write-Host "     cmake --build build --config Release -j" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  3. Run tests:" -ForegroundColor White
    Write-Host "     ctest --test-dir build --build-config Release --output-on-failure" -ForegroundColor Yellow
    Write-Host ""
    
    if (-not $SkipEmscripten) {
        Write-Host "  4. To build WebAssembly bindings:" -ForegroundColor White
        Write-Host "     $EmsdkDir\emsdk_env.bat" -ForegroundColor Yellow
        Write-Host "     emcmake cmake -B build-wasm -DBUILD_WASM=ON" -ForegroundColor Yellow
        Write-Host "     cmake --build build-wasm" -ForegroundColor Yellow
        Write-Host ""
    }
    
    Write-Info "For more information, see README.md"
}

# Run main
Main
