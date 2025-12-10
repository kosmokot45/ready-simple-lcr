[Setup]
AppName=LCR_Meter
AppVersion=1.0
DefaultDirName={pf}\LCR_Meter
DefaultGroupName=LCR_Meter
OutputBaseFilename=LCR_Meter_Installer
Compression=lzma
SolidCompression=yes
PrivilegesRequired=admin
WizardStyle=modern

[Files]
Source: "dist\LCR_Meter.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "templates\*"; DestDir: "{app}\templates"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\LCR_Meter"; Filename: "{app}\LCR_Meter.exe"
Name: "{commondesktop}\LCR_Meter"; Filename: "{app}\LCR_Meter.exe"

[Run]
Filename: "{app}\LCR_Meter.exe"; Description: "Запустить LCR_Meter"; Flags: nowait postinstall skipifsilent
