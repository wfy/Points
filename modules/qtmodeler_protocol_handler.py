import os
import sys
import urllib.parse
from modules.viewer import QTModelerViewer

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
        viewer = QTModelerViewer()
        viewer.open(las_path)

if __name__ == "__main__":
    main()
