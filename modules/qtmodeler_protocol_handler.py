import os
import sys
import urllib.parse
import subprocess

def find_qtmodeler():
    paths = [
        r"C:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"C:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"C:\QTModeler_840_UX\QTModeler.exe",
        r"D:\Program Files\QTModeler_820_UX_TRIAL\QTModeler.exe",
        r"D:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"D:\QTModeler_840_UX\QTModeler.exe",
        r"E:\Program Files\Applied Imagery\QT Modeler\QTModeler.exe",
        r"E:\QTModeler_840_UX\QTModeler.exe"
    ]
    for p in paths:
        if os.path.exists(p):
            return p
    return None

def main():
    if len(sys.argv) < 2:
        return
    raw = sys.argv[1]
    if raw.lower().startswith("qtmodeler:///"):
        raw = raw[13:]
    elif raw.lower().startswith("qtmodeler://"):
        raw = raw[12:]
    elif raw.lower().startswith("qtmodeler:"):
        raw = raw[10:]
    las_path = urllib.parse.unquote(raw).strip().replace("/", "\\")
    if las_path.endswith("\\"):
        las_path = las_path[:-1]
    
    if os.path.exists(las_path):
        qt = find_qtmodeler()
        if qt and os.path.exists(qt):
            subprocess.Popen([qt, las_path])
        else:
            os.startfile(las_path)

if __name__ == "__main__":
    main()
