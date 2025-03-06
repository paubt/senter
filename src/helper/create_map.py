import xml.etree.ElementTree as ET
import csv

WALL_CODE = 2

tree = ET.parse('../../assets/map_big.tmx')

root = tree.getroot()
data = root.find('.//data').text


o = list()
row = float(root.attrib['height'])
col = 0.

for l in str(data).splitlines():
    for e in l.split(sep=","):
        try:
            if int(e) == WALL_CODE:
                o.append((row, col))
                print(e + " Wall")
            else:
                print("empty")
            col = col + 1.
        except ValueError:
            print("empty")
    col = 0.
    row = row - 1.

print(o)
print(len(o))