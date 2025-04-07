import zipfile
import os

kmz_path = 'data/AA_City_Boundary.kmz'
extract_path = 'data/extracted_AA_City_Boundary'
with zipfile.ZipFile(kmz_path, 'r') as kmz:
    kmz.extractall(extract_path)

print(f"Extracted KMZ contents to {extract_path}")