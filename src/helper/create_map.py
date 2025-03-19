import xml.etree.ElementTree as ET
import csv

WALL_CODE = 5

tree = ET.parse('../../assets/map_small.tmx')

root = tree.getroot()
data = root.find('.//data').text


o = list()
row = float(root.attrib['height']) - 1.
col = 0.

for l in str(data).splitlines():
    if len(l) == 0:
        continue
    for e in l.split(sep=","):
        try:
            if int(e) == WALL_CODE:
                o.append((row, col))
                print("+",end="")
            else:
                print(" ", end="")
            col = col + 1.
        except ValueError:
            print("empty", end="")
    print("")
    col = 0.
    row = row - 1.

print(o)
print(len(o))