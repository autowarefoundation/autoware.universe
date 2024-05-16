#ifndef PREFIX
#define PREFIX "DUMMY"
#endif

#ifndef NAME
#define NAME "DUMMY"
#endif

#ifndef PREFIX
#define PREFIX "DUMMY"
#endif

#ifndef GEOIDDIR
#define GEOIDDIR "DUMMY"
#endif

#define Project "GeographicLib"
#define URL "https://geographiclib.sourceforge.io"
#define Version "1.0"

[Setup]
AppName={#Project} {#NAME}
AppVerName={#NAME} geoid data
AppPublisher={#URL}
AppPublisherURL={#URL}
AppSupportURL={#URL}/html/geoid.html
AppUpdatesURL=https://sourceforge.net/projects/geographiclib/files/geoids-distrib
DefaultDirName={commonappdata}\{#Project}
DisableDirPage=no
DisableProgramGroupPage=true
OutputBaseFilename={#PREFIX}
Compression=lzma/ultra64
DirExistsWarning=no
AllowUNCPath=true
AppendDefaultDirName=true
ShowLanguageDialog=no
OutputDir={#GEOIDDIR}\installers
PrivilegesRequired=none
VersionInfoVersion={#Version}
VersionInfoCompany={#Project}
VersionInfoDescription={#NAME} geoid data
VersionInfoTextVersion={#Version}
VersionInfoCopyright=Public Domain
AppVersion={#Version}
AppComments={#NAME} geoid data
AppContact=karney@alum.mit.edu
UninstallDisplayName={#Project} geoid {#NAME}
DiskSpanning=no

[Files]
Source: {#GEOIDDIR}\geoids\{#prefix}.*; DestDir: {app}\geoids; Flags: ignoreversion
