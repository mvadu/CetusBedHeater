/*
(Get-Content .\src\versionInfo.h | foreach-object { if($_.StartsWith("const VersionInfo")){
  $line = $_;
  $line -match "(?<version>\{.*Minor:\s+(?<minor>\d+).*BuildTime:\s+(?<buildtime>\S+)\})" | out-null;
  $line = $line.replace($matches['minor'],[int]$matches['minor']+1)
  $buildTime = [datetime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ss.sssZ')
  $line = $line.replace($matches['buildTime'],$buildTime)
  write-output $line
 } else {write-output $_}}) | Set-Content .\src\versionInfo.h
*/

struct VersionInfo
{
  uint8_t Major;
  uint8_t Minor;
  //ISO 8601 time
  const char* BuildTime;
};

const VersionInfo firmwareInfo = VersionInfo{    Major:  217,    Minor:  217,    BuildTime: "2020-06-18T04:05:45.896564" };
