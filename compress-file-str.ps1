[CmdletBinding()]
param (
    #Source file to be processed. Mandatory parameter
    [parameter(Mandatory = $true)]
    [alias("input")]
    [alias("Path")]
    [string]$source,
    
    #Target header file to store the compressed data. Optional parameter
    [parameter(Mandatory = $false)]
    [string]$target = $null,

    #Target variable name to store the compressed data. Optional parameter
    [parameter(Mandatory = $false)]
    [string]$targetVar = $null
)
if (!(test-path $source)) {
    write-error "Input file not found!!"
    return;
}

$sourceItem = Get-Item -Path $source
$msi = $sourceItem.OpenRead()

$mso = new-Object IO.MemoryStream
$gs = New-Object System.IO.Compression.GzipStream $mso, ([IO.Compression.CompressionMode]::Compress)
$msi.CopyTo($gs)
$gs.Dispose()
$out = $mso.ToArray()
$msi.Dispose()
$mso.Dispose()

$len = $out.length
$out = $($out.ForEach( {'0x{0:X2}' -f $_}) -join ",")

if ([String]::IsNullOrEmpty($target)) {
    Write-Output "Len: $len`n $out" 
}
else {
    if (!(test-path $target)) {
        new-item $target | Out-Null
    }
    
    if ([String]::IsNullOrEmpty($targetVar)) {
        $targetVar = $(Get-Item $source).Name.Replace('.', '_')      
    }

    $found = $false

    (get-content $target).foreach( {
            $line = $_ 
            if ($line -match $targetVar) {
                $line -replace "{[0-9A-Fx,].*}" , "{$out}; `n//Length:$len"
                $found = $true 
            }
            else {
                return $line
            }
        })  | Set-Content $target	      
    if (!$found) {
        Add-Content $target "`n const uint8_t $targetVar[] PROGMEM = {$out};`n//Length:$len"
    }
    Write-Output 'Done'
}