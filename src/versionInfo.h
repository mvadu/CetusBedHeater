/*
(Get-Content .\src\versionInfo.h | foreach-object { if($_.StartsWith("const VersionInfo")){
  $line = $_;
  $line -match "(?<version>\{.*Minor:\s+(?<minor>\d+).*BuildTime:\s+(?<buildtime>\d+)\})" | out-null;
  $line = $line.replace($matches['minor'],[int]$matches['minor']+1)
  $buildTime = [int64](([datetime]::UtcNow)-(get-date "1/1/1970")).TotalMilliseconds
  $line = $line.replace($matches['buildTime'],$buildTime)
  write-output $line
 } else {write-output $_}}) | Set-Content .\src\versionInfo.h
*/

struct VersionInfo
{
  uint8_t Major;
  uint8_t Minor;
  uint64_t  BuildTime;
};

const VersionInfo firmwareInfo = VersionInfo{    Major:  1,    Minor:  49,    BuildTime:  4984960049976};
