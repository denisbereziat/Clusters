import math
import pyproj
from shapely.geometry import LineString


def intersection(interval1, interval2):
    if interval2[0] > interval1[1] or interval1[0] > interval2[1]:
        return None
    else:
        return [max([interval1[0], interval2[0]]), min(interval1[1], interval2[1])]


def angle_btw_vectors(pt1, pt2, pt3):
    geodesic = pyproj.Geod(ellps='WGS84')
    fwd_azimuth1, back_azimuth1, _distance = geodesic.inv(pt1[1], pt1[0], pt2[1], pt2[0])
    fwd_azimuth2, back_azimuth2, _distance = geodesic.inv(pt2[1], pt2[0], pt3[1], pt3[0])
    angle = fwd_azimuth2 - fwd_azimuth1
    # angle = bearing(pt1, pt2) - bearing(pt2, pt3)
    # angle = angle*360/(2*math.pi)
    angle = abs(angle)
    if angle > 180:
        angle = 360 - 180
    return angle

# def angle_btw_vectors(v1, v2):
#     magnitude1 = dot(v1, v1) ** 0.5
#     magnitude2 = dot(v2, v2) ** 0.5
#     cos_v1v2 = dot(v1, v2) / (magnitude1 * magnitude2)
#     try:
#         angle = math.acos(cos_v1v2)
#     except ValueError:
#         angle = math.pi
#     angle = abs(math.degrees(angle))
#     return angle


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def distance(x1, y1, x2, y2):
    x1, y1 = x1*math.pi/180, y1*math.pi/180
    x2, y2 = x2*math.pi/180, y2*math.pi/180
    try:
        return math.acos(math.sin(y1)*math.sin(y2)+math.cos(y1)*math.cos(y2)*math.cos(x1-x2))* 6371009
    except ValueError:  
        return math.acos(math.cos(90-y1)*math.cos(90-y2)+math.sin(90-y1)*math.sin(90-y2)*math.cos(x1-x2))* 6371009


def edges_intersect(edge1, edge2, graph):
    l1 = LineString((graph.nodes[edge1[0]]["x"], graph.nodes[edge1[0]]["y"]), (graph.nodes[edge1[1]]["x"], graph.nodes[edge1[1]]["y"]))
    l2 = LineString((graph.nodes[edge2[0]]["x"], graph.nodes[edge2[0]]["y"]), (graph.nodes[edge2[1]]["x"], graph.nodes[edge2[1]]["y"]))
    return l1.intersects(l2)


def m_displacement_to_lat_lon(pt, dn, de):
    lat = pt[1]
    lon = pt[0]

    R = 6378137

    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*lat/180))

    latO = lat + dLat * 180/math.pi
    lonO = lon + dLon * 180/math.pi

    return lonO, latO


def point_on_edge(x, y, edge, graph):
    def line_eq(p1, p2):
        a = (p1[1]-p2[1])/(p1[0]-p2[0])
        b = p1[1] - a*p1[1]
        f = lambda x: a*x+b
        return f
    geometry = graph.edges[edge]['geometry']
    for i in range(len(geometry)-1, 2):
        f = line_eq((geometry[i],geometry[i+1]), (geometry[i+2],geometry[i+3]))
        if abs(f(x)-y) <= 0.0001:
            return True
    return False
