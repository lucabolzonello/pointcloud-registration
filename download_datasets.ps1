$ErrorActionPreference = "Stop"

$DataDir = "data\bunny"
$Url = "https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz"
$Archive = "$DataDir\bunny.tar.gz"

Write-Host "Creating data directory..."
New-Item -ItemType Directory -Force -Path $DataDir | Out-Null

Write-Host "Downloading Stanford Bunny dataset..."
Invoke-WebRequest -Uri $Url -OutFile $Archive

Write-Host "Extracting..."
tar -xzf $Archive -C $DataDir

Write-Host "Cleaning up..."
Remove-Item $Archive

Write-Host "Done! Dataset is in $DataDir"