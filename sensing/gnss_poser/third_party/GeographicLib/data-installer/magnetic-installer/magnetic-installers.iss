#ifndef PREFIX
#define PREFIX "DUMMY"
#endif

#ifndef NAME
#define NAME "DUMMY"
#endif

#ifndef MAGNETICDIR
#define MAGNETICDIR "DUMMY"
#endif

#define Project "GeographicLib"
#define URL "https://geographiclib.sourceforge.io"
#define Version "1.0"

[Setup]
AppName={#Project} {#NAME}
AppVerName={#NAME} magnetic model
AppPublisher={#URL}
AppPublisherURL={#URL}
AppSupportURL={#URL}/html/magnetic.html
AppUpdatesURL=https://sourceforge.net/projects/geographiclib/files/magnetic-distrib
DefaultDirName={commonappdata}\{#Project}
DisableDirPage=no
DisableProgramGroupPage=true
OutputBaseFilename={#PREFIX}
Compression=lzma/ultra64
DirExistsWarning=no
AllowUNCPath=true
AppendDefaultDirName=true
ShowLanguageDialog=no
OutputDir={#MAGNETICDIR}
PrivilegesRequired=none
VersionInfoVersion={#Version}
VersionInfoCompany={#Project}
VersionInfoDescription={#NAME} magnetic model
VersionInfoTextVersion={#Version}
VersionInfoCopyright=Public Domain
AppVersion={#Version}
AppComments={#NAME} magnetic model
AppContact=karney@alum.mit.edu
UninstallDisplayName={#Project} magnetic model {#NAME}
DiskSpanning=no

[Files]
Source: {#MAGNETICDIR}\magnetic\{#PREFIX}.wmm; DestDir: {app}\magnetic; Flags: ignoreversion
Source: {#MAGNETICDIR}\magnetic\{#PREFIX}.wmm.cof; DestDir: {app}\magnetic; Flags: ignoreversion
