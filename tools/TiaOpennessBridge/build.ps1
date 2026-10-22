# Builds the TIA Portal V18 Openness bridge (requires .NET SDK and TIA Portal V18 with Openness).
# Output: tools\TiaOpennessBridge\bin\Release\net48\TiaOpennessBridge.exe
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    dotnet build -c Release
} finally {
    Pop-Location
}
