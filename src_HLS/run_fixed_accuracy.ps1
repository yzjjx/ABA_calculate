param(
    [int]$DataWidth = 32,
    [int]$IntegerWidth = 16,
    [string]$InputFile = "in_txt/aba_input.txt"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$HlsRoot = "D:/Xilinx/Vitis_HLS/2023.1"
$Compiler = "D:/GCC_WIN64/mingw64/bin/g++.exe"
$Executable = Join-Path $ProjectRoot "build/fixed_accuracy_${DataWidth}_${IntegerWidth}.exe"

$Sources = @(
    "src_HLS/fixed_accuracy_test.cpp",
    "src_HLS/float_reference.cpp",
    "src_HLS/ABA_HLS.cpp",
    "src_HLS/T_R_out.cpp",
    "src_HLS/v_ori.cpp",
    "src_HLS/c.cpp",
    "src_HLS/cal_p.cpp",
    "src_HLS/back.cpp",
    "src_HLS/pass3.cpp",
    "src_HLS/ABA_parms.cpp",
    "src_HLS/block_mat.cpp"
)

Push-Location $ProjectRoot
try {
    & $Compiler -O2 -std=c++17 `
        "-DABA_DATA_W=$DataWidth" "-DABA_DATA_I=$IntegerWidth" `
        "-DABA_NATIVE_SIM" `
        "-Isrc_HLS" "-Iinclude" "-I$HlsRoot/include" `
        @Sources -o $Executable
    if ($LASTEXITCODE -ne 0) {
        throw "Fixed-point accuracy test compilation failed."
    }

    & $Executable $InputFile
    if ($LASTEXITCODE -ne 0) {
        throw "Fixed-point accuracy test failed."
    }
}
finally {
    Pop-Location
}
