#!/usr/bin/env python
"""Extract the buildings from OpenStreetMap data into
OpenSceneGraph-based 3D obstacles for INeT simulation.
- Shapely is used to compute the convex hulls of the ploygons
- Heights and colors are randomly picked

"""
from lxml import etree as et
from imposm.parser import OSMParser
from xml.dom import minidom
#from xml.etree import ElementTree as et
from shapely import geometry
import numpy as np
import os
import random
import sys

__author__ = "Chenguang Liu"
__version__ = "0.0.1"
__email__ = "liuchg@utexas.edu"

# Constants
K_HEIGHT_LST = np.arange(6, 18, 2)
K_COLORS_LST = [
    '203 65 84', '112 128 144', '153 51 0', '255 140 26', '153 0 51',
    '0 206 209', '255 204 0', '77 153 0', '221 160 221', '103 103 152',
    '148 184 184'
]
K_COLOR_BEACON = '0 250 154'
output_file = 'output.xml'


class CoordGeo(object):
    coords = {}

    def coords_callback(self, coords):
        for osmid, lon, lat in coords:
            self.coords[osmid] = (lat, lon)


class BuildingShapes(object):
    bldg_shp = []
    geomap = {}
    xml_elmts = []

    #cnt = 0

    def __init__(self, gmap):
        self.geomap = gmap

    def ways_callback(self, ways):
        for osmid, tags, refs in ways:
            if 'building' in tags:
                geo_contour = []
                for node_osmid in refs:
                    if node_osmid in self.geomap:
                        geo_contour.append(self.geomap[node_osmid])
                    # if self.cnt > 5:
                    #     break
                if len(geo_contour) > 2:
                    contour_str = ""
                    min_lat = 91
                    min_lon = 91
                    for latlon in geo_contour:
                        lat = latlon[0]
                        lon = latlon[1]
                        if lat < min_lat:
                            min_lat = lat
                        if lon < min_lon:
                            min_lon = lon
                    contour_str = compute_convex_polygon(geo_contour).replace(
                        ',', ' ')
                    # contour_str += ('  {}  {}'.format(lat, lon))
                    shape_str = 'prism {} {}'.format(rand_height(),
                                                     contour_str)
                    pos_str = 'min {} {} 0.1'.format(min_lat, min_lon)
                    ori_str = '0 0 0'
                    color_str = rand_color()
                    op_val = "0.5"
                    material_val = "brick"
                    shp_elmt = et.Element(
                        'object',
                        attrib={
                            'osmid': str(osmid),
                            'position': pos_str,
                            'orientation': ori_str,
                            'shape': shape_str,
                            'fill-color': color_str,
                            'material': material_val,
                            'opacity': op_val
                        })
                    self.xml_elmts.append(shp_elmt)
                    #self.cnt += 1


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def compute_convex_polygon(coord_lst):
    ploy = geometry.Polygon([p[0], p[1]] for p in coord_lst).convex_hull
    return ploy.wkt[10:-2]


def rand_color():
    return random.choice(K_COLORS_LST)


def rand_height():
    return random.choice(K_HEIGHT_LST)


def main():
    file_path = sys.argv[1]
    if not os.path.exists(file_path):
        print("{} doesn't exist.".format(file_path))
        exit(1)
    else:
        print("Extracting buildings from |{}|...".format(file_path))
        root = et.Element('environment')
        comment = et.Comment(
            'Generated obstacle configurations for {}'.format(file_path))
        root.append(comment)

        coord_parser = CoordGeo()
        p = OSMParser(
            concurrency=4, coords_callback=coord_parser.coords_callback)
        p.parse(file_path)

        bldg_parser = BuildingShapes(coord_parser.coords)

        p = OSMParser(concurrency=4, ways_callback=bldg_parser.ways_callback)
        p.parse(file_path)

        print('# Coords: {}'.format(len(coord_parser.coords)))
        print('# Buildings: {}'.format(len(bldg_parser.xml_elmts)))
        # Extent the children to the root element
        root.extend(bldg_parser.xml_elmts)
        #print(prettify(root))
        # Dump to file
        print('Writing result to {}'.format(output_file))
        output = et.ElementTree(root)
        output.write(output_file, pretty_print=True)


if __name__ == '__main__':
    print(' [Usage] osm_file_path')
    if len(sys.argv) < 2:
        print('Please provide file path to the osm file.')
        exit(1)
    main()
