!define PRODUCT_VERSION "1.1.7"
!define PRODUCT_NAME "Ascend AI Devkit Model Adapter"
!define PRODUCT_PUBLISHER "Huawei Technologies Co., Ltd."
!define PRODUCT_WEB_SITE "https://www.hiascend.com/"
!define PRODUCT_UNINST_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PRODUCT_NAME}"
!define PRODUCT_UNINST_ROOT_KEY "HKLM"
!define PRODUCT_RUN_ROOT_KEY 'HKCU "Software\Microsoft\Windows NT\CurrentVersion\AppCompatFlags\Layers"'

SetCompressor lzma

!include "MUI.nsh"
!include LogicLib.nsh
!include "WordFunc.nsh"

!define MUI_ABORTWARNING
!define MUI_ICON "src\labelme\labelme\icons\ascend_32_32.ico"
!define MUI_UNICON "src\labelme\labelme\icons\ascend_32_32.ico"

; !define MUI_WELCOMEPAGE_TITLE "Welcome to Ascend AI Devkit Model Adapter ${PRODUCT_VERSION} Setup"
; !insertmacro MUI_PAGE_WELCOME

; !insertmacro MUI_PAGE_LICENSE "D:\gitee_code\ascend-devkit\src\labelme\dist\__main__\src\labelme\LICENSE"

; !insertmacro MUI_PAGE_DIRECTORY

!insertmacro MUI_PAGE_INSTFILES

!define MUI_FINISHPAGE_TITLE "Completing Ascend AI Devkit Model Adapter ${PRODUCT_VERSION} Setup"
!define MUI_FINISHPAGE_RUN "$INSTDIR\${PRODUCT_NAME}.exe"
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_INSTFILES


!insertmacro MUI_LANGUAGE "English"


!insertmacro MUI_RESERVEFILE_INSTALLOPTIONS

Name "${PRODUCT_NAME}"
OutFile "Ascend-devkit-model-adapter_${PRODUCT_VERSION}_win-x86_64_update.exe"
InstallDir "$PROGRAMFILES\Ascend AI Devkit Model Adapter"
ShowInstDetails show
ShowUnInstDetails show
RequestExecutionLevel admin

Section "MainSection" SEC01
  SetOutPath "$INSTDIR\*.*"
  SetOverwrite ifnewer
  File /r "*.*"
SectionEnd

Section -AdditionalIcons
  SetOutPath $INSTDIR
  CreateDirectory "$SMPROGRAMS\${PRODUCT_NAME}"
  CreateShortCut "$SMPROGRAMS\${PRODUCT_NAME}\${PRODUCT_NAME}.lnk" "$INSTDIR\${PRODUCT_NAME}.exe"
SectionEnd

Section -Post
  WriteUninstaller "$INSTDIR\uninstall.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "InstallPath" "$INSTDIR"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayIcon" "$INSTDIR\${PRODUCT_NAME}.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayName" "$(^Name)"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "UninstallString" "$INSTDIR\uninstall.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayVersion" "${PRODUCT_VERSION}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "URLInfoAbout" "${PRODUCT_WEB_SITE}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "Publisher" "${PRODUCT_PUBLISHER}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "Installed" "1"
  WriteRegStr "HKCU" "Software\Microsoft\Windows NT\CurrentVersion\AppCompatFlags\Layers" "$INSTDIR\${PRODUCT_NAME}.exe" "~ RUNASADMIN"
SectionEnd

Section Uninstall
  nsExec::Exec "$INSTDIR\src\labelme\envs\env_uninst.bat"

  Delete "$INSTDIR\uninstall.exe"

  Delete "$SMPROGRAMS\${PRODUCT_NAME}\${PRODUCT_NAME}.lnk"

  RMDir "$SMPROGRAMS\${PRODUCT_NAME}"

  RMDir /r "$INSTDIR\*.*"

  RMDir "$INSTDIR"

  DeleteRegKey ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}"
  DeleteRegValue ${PRODUCT_RUN_ROOT_KEY} "$INSTDIR\${PRODUCT_NAME}.exe"
  SetAutoClose true
SectionEnd

Function .onInit
  System::Call 'kernel32::CreateMutexA(i 0, i 0, t "Ascend-devkit-model-adapter_${PRODUCT_VERSION}_win-x86_64_update.exe") i .r1 ?e'
  Pop $R0
  StrCmp $R0 0 +3
  MessageBox MB_OK|MB_USERICON|MB_TOPMOST "Setup is already running."
  Abort

  ReadRegStr $0 ${PRODUCT_UNINST_ROOT_KEY} '${PRODUCT_UNINST_KEY}' "Installed"
  ${If} $0 != "1"
    MessageBox MB_OK|MB_USERICON|MB_TOPMOST '$(^Name) is not Installed on your computer. It needs to be installed first.'
    Quit
  ${EndIf}

  nsProcess::_FindProcess "${PRODUCT_NAME}.exe"
  Pop $R0
  ${If} $R0 == 0
    MessageBox MB_OK|MB_USERICON|MB_TOPMOST "$(^Name) Running, please stop the program first and try again."
    Quit
  ${EndIf}

  ReadRegStr $0 ${PRODUCT_UNINST_ROOT_KEY} '${PRODUCT_UNINST_KEY}' "DisplayVersion"
  ${VersionCompare}  "$0"  "${PRODUCT_VERSION}"  $1
  StrCmp $1  "1"  0  +3
  MessageBox MB_OK  "Ascend AI Devkit Model Adapter is the latest version."
  Abort
  StrCmp $1  "0"  0  +3
  MessageBox MB_OK  "Ascend AI Devkit Model Adapter is the latest version."
  Abort
FunctionEnd

Function un.onInit
  nsProcess::_FindProcess "${PRODUCT_NAME}.exe"
  Pop $R0
  IntCmp $R0 0 +1 no_run no_run
  MessageBox MB_OK|MB_USERICON|MB_TOPMOST "$(^Name) Running, please stop the program first and try again."
  Quit
  no_run:
  nsProcess::_Unload
  MessageBox MB_ICONQUESTION|MB_YESNO|MB_DEFBUTTON2|MB_TOPMOST "Are you sure you want to completely remove $(^Name) and all its components?" IDYES +2
  Abort
FunctionEnd

Function un.onUninstSuccess
  HideWindow
  MessageBox MB_ICONINFORMATION|MB_OK|MB_TOPMOST "$(^Name) was successfully removed from your computer."
FunctionEnd
