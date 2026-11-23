# ESP-IDF environment for ESP32-C3 builds (this machine).
# Usage:  . .\buildenv.ps1
# The IDF install is C:\Espressif with tools under C:\Espressif\tools and the
# python venv next to it. ESP_IDF_VERSION must be exported or idf_component_manager
# crashes with "expected string or bytes-like object, got 'NoneType'".

$env:IDF_TOOLS_PATH = 'C:\Espressif'
$env:IDF_PATH       = 'C:\Espressif\v6.0.1\esp-idf'
$env:ESP_IDF_VERSION = '6.0.1'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\python_env\idf6.0_py3.13_env'
# esp_rom's gdbinit generation needs this (export scripts normally set it).
$env:ESP_ROM_ELF_DIR = 'C:\Espressif\tools\esp-rom-elfs\20241011'

$venv = Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts'
$riscv = 'C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin'
$cmake = 'C:\Espressif\tools\cmake\4.0.3\bin'
$ninja = 'C:\Espressif\tools\ninja\1.12.1'
$idfexe = 'C:\Espressif\tools\idf-exe\1.0.3'

foreach ($p in @($venv, $riscv, $cmake, $ninja, $idfexe)) {
    if ((Test-Path $p) -and ($env:PATH -notlike "*$p*")) {
        $env:PATH = "$p;$env:PATH"
    }
}

function Invoke-Idf {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$IdfArgs)
    & (Join-Path $venv 'python.exe') (Join-Path $env:IDF_PATH 'tools\idf.py') @IdfArgs
}

Write-Host "IDF_PATH       = $env:IDF_PATH"
Write-Host "ESP_IDF_VERSION= $env:ESP_IDF_VERSION"
