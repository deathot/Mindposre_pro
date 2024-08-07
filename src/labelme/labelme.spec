# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(
    ['..\\..\\src\\labelme\\labelme\\__main__.py'],
    pathex=['..\\..\\..\\ascend-devkit'],
    binaries=[],
    datas=[
        ('../../src/labelme', 'src/labelme'),
        ('../../src/models_adaption', 'src/models_adaption'),
    ],
    hiddenimports=['matplotlib.backends.backend_tkagg'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='Ascend AI Devkit Model Adapter',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='labelme/icons/ascend_32_32.ico',
)
coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='Ascend AI Devkit Model Adapter',
)
