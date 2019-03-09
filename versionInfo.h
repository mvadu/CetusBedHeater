/*
(Get-Content .\versionInfo.h | foreach-object { if($_.StartsWith("const VersionInfo")){
    $line = $_;  
    $line -match "(?<version>\{.*\})" | out-null;
    $version = $matches['version']|ConvertFrom-Json;
    $version.Minor += 1; $version.BuildTime = [int64](([datetime]::UtcNow)-(get-date "1/1/1970")).TotalMilliseconds
    $line = $line -replace "(?<version>\{.*\})", $(((($version | ConvertTo-Json) -replace "`"","") -split "`r`n") -join "")
    write-output $line
} else {write-output $_}}) | Set-Content .\versionInfo.h 
*/

struct VersionInfo
{
  uint8_t Major;
  uint8_t Minor;  
  uint64_t  BuildTime;
};

const VersionInfo firmwareInfo = VersionInfo{    Major:  1,    Minor:  8,    BuildTime:  1552153960649};
