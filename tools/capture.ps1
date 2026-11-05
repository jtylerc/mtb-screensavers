# Runs a screensaver (.scr) in preview mode inside an off-screen window and saves frames at the given times.
param([string]$Scr, [string]$Out, [int]$W = 1600, [int]$H = 900, [string]$Times = "2")
Add-Type -AssemblyName System.Windows.Forms, System.Drawing
Add-Type @"
using System; using System.Runtime.InteropServices;
public static class PW { [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr dc, uint f);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware(); }
"@
[PW]::SetProcessDPIAware() | Out-Null
$f = New-Object System.Windows.Forms.Form
$f.FormBorderStyle = 'None'; $f.StartPosition = 'Manual'; $f.ShowInTaskbar = $false
$f.Location = New-Object System.Drawing.Point(-5000, -5000); $f.ClientSize = New-Object System.Drawing.Size($W, $H)
$f.Show()
# start it directly: going through the .scr file association would force /S (full screen)
$si = New-Object Diagnostics.ProcessStartInfo; $si.FileName = $Scr; $si.Arguments = "/p " + $f.Handle.ToInt64(); $si.UseShellExecute = $false
$p = [Diagnostics.Process]::Start($si)
$sw = [Diagnostics.Stopwatch]::StartNew()
foreach ($t in ($Times -split ";" | ForEach-Object { [double]::Parse($_, [Globalization.CultureInfo]::InvariantCulture) })) {
  while ($sw.Elapsed.TotalSeconds -lt $t) { [System.Windows.Forms.Application]::DoEvents(); Start-Sleep -Milliseconds 10 }
  $bmp = New-Object System.Drawing.Bitmap($W, $H); $g = [System.Drawing.Graphics]::FromImage($bmp); $dc = $g.GetHdc()
  [PW]::PrintWindow($f.Handle, $dc, 2) | Out-Null; $g.ReleaseHdc($dc)
  $bmp.Save((Join-Path $Out ("t{0:00.0}.png" -f $t)), [System.Drawing.Imaging.ImageFormat]::Png); $g.Dispose(); $bmp.Dispose()
}
"alive: " + (-not $p.HasExited); if ($p.HasExited) { "exit code: 0x{0:X8}" -f $p.ExitCode }
if (-not $p.HasExited) { Stop-Process -Id $p.Id }
$f.Close()
