import os
import h3
import shapely as shp
from h3 import geo_to_h3shape
from shapely.geometry.polygon import Polygon
from shapely.geometry.multipolygon import MultiPolygon
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np

extract_path = 'data/extracted_AA_City_Boundary'
kml_file = os.path.join(extract_path, 'doc.kml')
geo_data = gpd.read_file(kml_file, driver='KML')
if geo_data.crs != "EPSG:4326":
    geo_data = geo_data.to_crs("EPSG:4326")
geo_data.boundary.plot(edgecolor="blue", linewidth=2)
plt.axis('equal')
plt.show()

map_multipolygon: MultiPolygon = geo_data.union_all()
h3_map_polygon = geo_to_h3shape(map_multipolygon)
hexagons = h3.h3shape_to_cells(h3_map_polygon, 10) # 6 - 10 work well
hexagons_geo_dict = h3.cells_to_geo(hexagons)
coordinates = hexagons_geo_dict['coordinates']
x_coordinates = []
y_coordinates = []
for region_lines in coordinates:
    region_x, region_y = zip(*region_lines)
    plt.plot(region_x, region_y, color='blue')
plt.xlabel('Longitude', fontsize=16)
plt.ylabel('Latitude', fontsize=16)
plt.axis('equal')
plt.show()
