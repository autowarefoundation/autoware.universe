#ifndef PREFIX
#define PREFIX "DUMMY"
#endif

#ifndef NAME
#define NAME "DUMMY"
#endif

#ifndef PREFIX
#define PREFIX "DUMMY"
#endif

#ifndef GRAVITYDIR
#define GRAVITYDIR "DUMMY"
#endif

#define Project "GeographicLib"
#define URL "https://geographiclib.sourceforge.io"
#define Version "1.0"

[Setup]
AppName={#Project} {#NAME}
AppVerName={#NAME} gravity model
AppPublisher={#URL}
AppPublisherURL={#URL}
AppSupportURL={#URL}/html/gravity.html
AppUpdatesURL=https://sourceforge.net/projects/geographiclib/files/gravity-distrib
DefaultDirName={commonappdata}\{#Project}
DisableDirPage=no
DisableProgramGroupPage=true
OutputBaseFilename={#PREFIX}
Compression=lzma/ultra64
DirExistsWarning=no
AllowUNCPath=true
AppendDefaultDirName=true
ShowLanguageDialog=no
OutputDir={#GRAVITYDIR}\gravity-installer
PrivilegesRequired=none
VersionInfoVersion={#Version}
VersionInfoCompany={#Project}
VersionInfoDescription={#NAME} gravity model
VersionInfoTextVersion={#Version}
VersionInfoCopyright=Public Domain
AppVersion={#Version}
AppComments={#NAME} gravity model
AppContact=karney@alum.mit.edu
UninstallDisplayName={#Project} gravity model {#NAME}
DiskSpanning=no

[Files]
Source: {#GRAVITYDIR}\gravity\{#prefix}.egm; DestDir: {app}\gravity; Flags: ignoreversion
Source: {#GRAVITYDIR}\gravity\{#prefix}.egm.cof; DestDir: {app}\gravity; Flags: ignoreversion
