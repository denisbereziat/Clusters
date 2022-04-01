import math
import pyproj
from shapely.geometry import LineString
import matplotlib.pyplot as plt

def hash_graph(graph, resolution):
    # TODO
    # hash_nodes = [[[] for i in range(resolution)] for i in range(resolution)]
    hash_nodes = dict()
    hash_edges = dict()
    edge_hash_tab=[]
    max_y, min_y = -1000, 1000
    max_x, min_x = -1000, 1000
    for node in graph.nodes:
        x = graph.nodes[node]["x"]
        y = graph.nodes[node]["y"]
        max_x = max(max_x, x)
        min_x = min(min_x, x)
        max_y = max(max_y, y)
        min_y = min(min_y, y)
    # for edge in graph.edges:
    #     node1 = edge[0]
    #     node2 = edge[1]
    #     x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) /2
    #     y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) /2
    #     middle_coords = (x_middle, y_middle)
    #     max_x = max(max_x, x_middle)
    #     min_x = min(min_x, x_middle)
    #     max_y = max(max_y, y_middle)
    #     min_y = min(min_y, y_middle)
    #     edge_hash_tab.append([edge, middle_coords])
    x_step = round((max_x-min_x) / resolution, 8)
    y_step = round((max_y-min_y) / resolution, 8)
    # print("max", max_x, max_y)
    # print("min", min_x, min_y)
    # print("step :", x_step, y_step)

    for node in graph.nodes:
        h = hash_pos(graph.nodes[node]["x"], graph.nodes[node]["y"], min_x, min_y, x_step, y_step)
        if h in hash_nodes:
            hash_nodes[h].append(node)
        else:
            hash_nodes[h] = [node]

    for edge in graph.edges:
        node1 = edge[0]
        node2 = edge[1]
        x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) / 2
        y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) / 2
        h = hash_pos(x_middle, y_middle, min_x, min_y, x_step, y_step)
        if h in hash_edges:
            hash_edges[h].append(edge)
        else:
            hash_edges[h] = [edge]
    return hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution


def hash_pos(x, y, min_x, min_y, step_x, step_y):
    x_pos = (x - min_x) // step_x
    y_pos = (y - min_y) // step_y
    return x_pos, y_pos


def extract_deposit_times(filepath):
    """Extract just the flight plans deposit times"""
    deposit_times_list = []
    with open(filepath) as f:
        for line in f.readlines():
            deposit_time = line[0:8]
            if deposit_time not in deposit_times_list:
                deposit_times_list.append(deposit_time)
    # print(deposit_times_list)
    for i in range(len(deposit_times_list)):
        l = deposit_times_list[i].split(':')
        deposit_time_in_s = 3600 * int(l[0]) + 60 * int(l[1]) + int(l[2])
        deposit_times_list[i] = deposit_time_in_s
    # print(deposit_times_list)
    return deposit_times_list


def find_closest_node_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution, graph):
    list = find_list_of_closest_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution)
    return find_closest_node_in_list(x, y, list, graph)


def find_list_of_closest_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution):
    h = hash_pos(x, y, min_x, min_y, step_x, step_y)
    possible_closest = []
    found = False
    radius = 1
    while not found:
        if radius > resolution:
            raise Exception
        for x in range(-radius, radius+1):
            for y in range(-radius, radius+1):
                _h = (h[0] + x, h[1] + y)
                if _h in hash_map:
                    # TODO ON AJOUTE DES DOUBLONS LA
                    possible_closest += hash_map[_h]
                    found = True
        radius += 1
    return possible_closest


def find_closest_node_in_list(x, y, nodes_list, graph):
    closest = None
    smallest_dst = math.inf
    for node in nodes_list:
        dist = distance(x, y, graph.nodes[node]["x"], graph.nodes[node]["y"])
        if dist < smallest_dst:
            closest = node
            smallest_dst = dist
    if closest is None:
        print("no node found")
        raise Exception
    else:
        return closest


def find_closest_edge_in_list(x, y, edge_list, graph):
    # TODO AJOUTER UNE DIST MINI POUR SAVOIR SI DANS UNCONSTRAINED OU CONSTRAIND ABD(dist) < precision
    # PARCE QUE SI ON EST TRES PROCHE DE l'ARRETE ON VEUT FAIRE COMME CONSTRAINED ET SINON  COMME UNCONSTRAUNED
    closest = None
    smallest_dist = math.inf
    for edge in edge_list:
        node1 = edge[0]
        node2 = edge[1]
        x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) / 2
        y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) / 2
        dist = distance(x, y, x_middle, y_middle)
        if dist < smallest_dist:
            closest = edge
            smallest_dist = dist
    if closest is None:
        print("no node found")
        raise Exception
    else:
        return closest, smallest_dist


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
        angle = 360 - angle
    return angle


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def distance(x1, y1, x2, y2):
    g = pyproj.Geod(ellps="WGS84")
    dist = g.line_lengths([x1,x2], [y1,y2])[0]
    return dist


def edges_intersect(edge1, edge2, graph):
    l1 = LineString((graph.nodes[edge1[0]]["x"], graph.nodes[edge1[0]]["y"]), (graph.nodes[edge1[1]]["x"], graph.nodes[edge1[1]]["y"]))
    l2 = LineString((graph.nodes[edge2[0]]["x"], graph.nodes[edge2[0]]["y"]), (graph.nodes[edge2[1]]["x"], graph.nodes[edge2[1]]["y"]))
    return l1.intersects(l2)


def scatter_graph(graph, marker='+', color='grey', show_edges=False):
    x = [graph.nodes[node]["x"] for node in graph.nodes]
    y = [graph.nodes[node]["y"] for node in graph.nodes]
    plt.scatter(x,y, marker=marker, color=color)
    if show_edges:
        for edge in graph.edges:
            x = [graph.nodes[edge[0]]["x"],graph.nodes[edge[1]]["x"]]
            y = [graph.nodes[edge[0]]["y"],graph.nodes[edge[1]]["y"]]
            plt.plot(x,y, color=color)


def plot_traj(traj, graph, marker='o', color='red'):
    x = [graph.nodes[node]["x"] for node in traj]
    y = [graph.nodes[node]["y"] for node in traj]
    plt.plot(x,y, marker=marker, color=color)


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

# TODO a faire au propre
unconstrained_node_dict = {'011032': None, '011033': None, '011034': None, '011035': None, '012028': None,
                           '012029': None, '012030': None, '012031': None, '012032': None, '012033': None,
                           '012034': None, '012035': None, '012036': None, '012037': None, '012038': None,
                           '012039': None, '013026': None, '013027': None, '013028': None, '013029': None,
                           '013030': None, '013031': None, '013032': None, '013033': None, '013034': None,
                           '013035': None, '013037': None, '013038': None, '013039': None, '013040': None,
                           '013041': None, '014024': None, '014025': None, '014026': None, '014027': None,
                           '014028': None, '014029': None, '014030': None, '014031': None, '014032': None,
                           '014033': None, '014034': None, '014035': None, '014037': None, '014038': None,
                           '014039': None, '014040': None, '014041': None, '014042': None, '015023': None,
                           '015024': None, '015025': None, '015026': None, '015027': None, '015028': None,
                           '015029': None, '015030': None, '015031': None, '015032': None, '015033': None,
                           '015034': None, '015035': None, '015038': None, '015039': None, '015040': None,
                           '015041': None, '015042': None, '015043': None, '015044': None, '016021': None,
                           '016022': None, '016023': None, '016024': None, '016025': None, '016026': None,
                           '016027': None, '016028': None, '016029': None, '016030': None, '016031': None,
                           '016032': None, '016033': None, '016034': None, '016035': None, '016036': None,
                           '016037': None, '016038': None, '016039': None, '016040': None, '016041': None,
                           '016042': None, '016043': None, '016044': None, '016045': None, '017020': None,
                           '017021': None, '017022': None, '017023': None, '017024': None, '017025': None,
                           '017026': None, '017027': None, '017028': None, '017029': None, '017030': None,
                           '017031': None, '017032': None, '017033': None, '017034': None, '017036': None,
                           '017037': None, '017038': None, '017039': None, '017040': None, '017041': None,
                           '017042': None, '017043': None, '017044': None, '017045': None, '017046': None,
                           '018019': None, '018020': None, '018021': None, '018022': None, '018023': None,
                           '018024': None, '018025': None, '018026': None, '018027': None, '018028': None,
                           '018029': None, '018030': None, '018031': None, '018032': None, '018033': None,
                           '018034': None, '018037': None, '018038': None, '018039': None, '018040': None,
                           '018041': None, '018042': None, '018043': None, '018044': None, '018045': None,
                           '018046': None, '018047': None, '019019': None, '019020': None, '019021': None,
                           '019022': None, '019023': None, '019024': None, '019025': None, '019026': None,
                           '019027': None, '019028': None, '019029': None, '019030': None, '019031': None,
                           '019032': None, '019033': None, '019034': None, '019035': None, '019036': None,
                           '019037': None, '019038': None, '019039': None, '019040': None, '019041': None,
                           '019042': None, '019043': None, '019044': None, '019045': None, '019046': None,
                           '019047': None, '019048': None, '020018': None, '020019': None, '020020': None,
                           '020021': None, '020022': None, '020023': None, '020024': None, '020025': None,
                           '020026': None, '020027': None, '020028': None, '020029': None, '020030': None,
                           '020031': None, '020032': None, '020033': None, '020034': None, '020035': None,
                           '020036': None, '020037': None, '020038': None, '020039': None, '020040': None,
                           '020041': None, '020042': None, '020043': None, '020044': None, '020045': None,
                           '020046': None, '020047': None, '020048': None, '020049': None, '021017': None,
                           '021018': None, '021019': None, '021020': None, '021021': None, '021022': None,
                           '021023': None, '021024': None, '021025': None, '021026': None, '021027': None,
                           '021028': None, '021029': None, '021030': None, '021031': None, '021032': None,
                           '021033': None, '021034': None, '021035': None, '021036': None, '021037': None,
                           '021038': None, '021039': None, '021040': None, '021041': None, '021042': None,
                           '021043': None, '021044': None, '021045': None, '021046': None, '021047': None,
                           '021048': None, '021049': None, '022017': None, '022018': None, '022019': None,
                           '022020': None, '022021': None, '022022': None, '022023': None, '022024': None,
                           '022025': None, '022026': None, '022027': None, '022028': None, '022029': None,
                           '022030': None, '022031': None, '022032': None, '022033': None, '022034': None,
                           '022035': None, '022036': None, '022037': None, '022038': None, '022039': None,
                           '022040': None, '022041': None, '022042': None, '022043': None, '022044': None,
                           '022045': None, '022046': None, '022047': None, '022048': None, '022049': None,
                           '022050': None, '023016': None, '023017': None, '023018': None, '023019': None,
                           '023020': None, '023021': None, '023022': None, '023023': None, '023024': None,
                           '023025': None, '023026': None, '023027': None, '023028': None, '023029': None,
                           '023030': None, '023031': None, '023032': None, '023033': None, '023034': None,
                           '023035': None, '023036': None, '023037': None, '023038': None, '023039': None,
                           '023040': None, '023041': None, '023042': None, '023043': None, '023044': None,
                           '023045': None, '023046': None, '023047': None, '023048': None, '023049': None,
                           '023050': None, '023051': None, '024015': None, '024016': None, '024017': None,
                           '024018': None, '024019': None, '024020': None, '024021': None, '024022': None,
                           '024023': None, '024024': None, '024025': None, '024026': None, '024027': None,
                           '024028': None, '024029': None, '024030': None, '024031': None, '024032': None,
                           '024033': None, '024034': None, '024035': None, '024036': None, '024037': None,
                           '024038': None, '024039': None, '024040': None, '024041': None, '024042': None,
                           '024043': None, '024044': None, '024045': None, '024046': None, '024047': None,
                           '024048': None, '024049': None, '024050': None, '024051': None, '024052': None,
                           '025015': None, '025016': None, '025017': None, '025018': None, '025019': None,
                           '025020': None, '025021': None, '025022': None, '025023': None, '025024': None,
                           '025025': None, '025026': None, '025027': None, '025028': None, '025029': None,
                           '025030': None, '025031': None, '025032': None, '025033': None, '025034': None,
                           '025035': None, '025036': None, '025037': None, '025038': None, '025039': None,
                           '025040': None, '025041': None, '025042': None, '025043': None, '025044': None,
                           '025045': None, '025046': None, '025047': None, '025048': None, '025049': None,
                           '025050': None, '025051': None, '025052': None, '026014': None, '026015': None,
                           '026016': None, '026017': None, '026018': None, '026019': None, '026020': None,
                           '026021': None, '026022': None, '026023': None, '026024': None, '026025': None,
                           '026026': None, '026027': None, '026028': None, '026029': None, '026030': None,
                           '026031': None, '026032': None, '026033': None, '026034': None, '026035': None,
                           '026036': None, '026037': None, '026038': None, '026039': None, '026040': None,
                           '026041': None, '026042': None, '026043': None, '026044': None, '026045': None,
                           '026046': None, '026047': None, '026048': None, '026049': None, '026050': None,
                           '026051': None, '026052': None, '027014': None, '027015': None, '027016': None,
                           '027017': None, '027018': None, '027019': None, '027020': None, '027021': None,
                           '027022': None, '027023': None, '027024': None, '027025': None, '027026': None,
                           '027027': None, '027028': None, '027029': None, '027031': None, '027032': None,
                           '027033': None, '027036': None, '027037': None, '027038': None, '027039': None,
                           '027040': None, '027041': None, '027042': None, '027043': None, '027044': None,
                           '027045': None, '027046': None, '027047': None, '027048': None, '027049': None,
                           '027050': None, '027051': None, '027052': None, '027053': None, '028013': None,
                           '028014': None, '028015': None, '028016': None, '028017': None, '028018': None,
                           '028019': None, '028020': None, '028021': None, '028022': None, '028023': None,
                           '028024': None, '028025': None, '028026': None, '028027': None, '028028': None,
                           '028029': None, '028031': None, '028032': None, '028037': None, '028038': None,
                           '028040': None, '028041': None, '028042': None, '028043': None, '028044': None,
                           '028045': None, '028046': None, '028047': None, '028048': None, '028049': None,
                           '028050': None, '028051': None, '028052': None, '028053': None, '028054': None,
                           '029013': None, '029014': None, '029015': None, '029016': None, '029017': None,
                           '029018': None, '029019': None, '029020': None, '029021': None, '029022': None,
                           '029023': None, '029024': None, '029025': None, '029026': None, '029027': None,
                           '029028': None, '029029': None, '029030': None, '029031': None, '029032': None,
                           '029037': None, '029038': None, '029040': None, '029041': None, '029042': None,
                           '029043': None, '029044': None, '029045': None, '029046': None, '029047': None,
                           '029048': None, '029049': None, '029050': None, '029051': None, '029052': None,
                           '029053': None, '029054': None, '030012': None, '030013': None, '030014': None,
                           '030017': None, '030018': None, '030019': None, '030020': None, '030021': None,
                           '030022': None, '030023': None, '030024': None, '030025': None, '030026': None,
                           '030027': None, '030038': None, '030039': None, '030040': None, '030041': None,
                           '030042': None, '030043': None, '030044': None, '030045': None, '030046': None,
                           '030047': None, '030048': None, '030049': None, '030050': None, '030051': None,
                           '030052': None, '030053': None, '030054': None, '031012': None, '031013': None,
                           '031014': None, '031016': None, '031017': None, '031018': None, '031019': None,
                           '031020': None, '031021': None, '031022': None, '031023': None, '031024': None,
                           '031025': None, '031026': None, '031038': None, '031039': None, '031040': None,
                           '031041': None, '031042': None, '031043': None, '031044': None, '031045': None,
                           '031046': None, '031047': None, '031048': None, '031049': None, '031050': None,
                           '031051': None, '031052': None, '031053': None, '031054': None, '031055': None,
                           '032011': None, '032012': None, '032013': None, '032014': None, '032015': None,
                           '032016': None, '032017': None, '032018': None, '032019': None, '032020': None,
                           '032021': None, '032022': None, '032023': None, '032024': None, '032025': None,
                           '032026': None, '032039': None, '032040': None, '032041': None, '032042': None,
                           '032043': None, '032044': None, '032045': None, '032046': None, '032047': None,
                           '032048': None, '032049': None, '032050': None, '032051': None, '032052': None,
                           '032053': None, '032054': None, '032055': None, '033011': None, '033012': None,
                           '033013': None, '033014': None, '033015': None, '033016': None, '033017': None,
                           '033018': None, '033019': None, '033020': None, '033021': None, '033022': None,
                           '033023': None, '033024': None, '033025': None, '033039': None, '033040': None,
                           '033041': None, '033042': None, '033043': None, '033044': None, '033045': None,
                           '033046': None, '033047': None, '033048': None, '033049': None, '033050': None,
                           '033051': None, '033052': None, '033053': None, '033054': None, '033055': None,
                           '034011': None, '034012': None, '034013': None, '034014': None, '034018': None,
                           '034022': None, '034023': None, '034024': None, '034025': None, '034026': None,
                           '034039': None, '034040': None, '034041': None, '034042': None, '034045': None,
                           '034046': None, '034047': None, '034048': None, '034049': None, '034050': None,
                           '034051': None, '034052': None, '034053': None, '034054': None, '034055': None,
                           '034056': None, '035010': None, '035011': None, '035012': None, '035013': None,
                           '035014': None, '035015': None, '035018': None, '035023': None, '035025': None,
                           '035040': None, '035041': None, '035046': None, '035047': None, '035048': None,
                           '035049': None, '035050': None, '035051': None, '035052': None, '035053': None,
                           '035054': None, '035055': None, '035056': None, '036010': None, '036011': None,
                           '036012': None, '036013': None, '036014': None, '036015': None, '036016': None,
                           '036017': None, '036018': None, '036025': None, '036026': None, '036040': None,
                           '036041': None, '036046': None, '036047': None, '036048': None, '036049': None,
                           '036050': None, '036051': None, '036052': None, '036053': None, '036054': None,
                           '036055': None, '036056': None, '037010': None, '037011': None, '037012': None,
                           '037013': None, '037014': None, '037015': None, '037016': None, '037017': None,
                           '037018': None, '037024': None, '037025': None, '037040': None, '037041': None,
                           '037045': None, '037046': None, '037047': None, '037048': None, '037049': None,
                           '037050': None, '037051': None, '037052': None, '037053': None, '037054': None,
                           '037055': None, '037056': None, '037057': None, '038009': None, '038010': None,
                           '038011': None, '038012': None, '038013': None, '038014': None, '038015': None,
                           '038016': None, '038017': None, '038018': None, '038025': None, '038026': None,
                           '038041': None, '038042': None, '038044': None, '038045': None, '038046': None,
                           '038047': None, '038048': None, '038049': None, '038050': None, '038051': None,
                           '038052': None, '038053': None, '038054': None, '038055': None, '038056': None,
                           '038057': None, '039010': None, '039011': None, '039012': None, '039013': None,
                           '039014': None, '039015': None, '039016': None, '039017': None, '039018': None,
                           '039042': None, '039043': None, '039044': None, '039045': None, '039046': None,
                           '039047': None, '039048': None, '039049': None, '039050': None, '039051': None,
                           '039052': None, '039053': None, '039054': None, '039055': None, '039056': None,
                           '039057': None, '040009': None, '040010': None, '040011': None, '040012': None,
                           '040013': None, '040014': None, '040015': None, '040016': None, '040017': None,
                           '040018': None, '040042': None, '040043': None, '040044': None, '040045': None,
                           '040046': None, '040047': None, '040048': None, '040049': None, '040050': None,
                           '040051': None, '040052': None, '040053': None, '040054': None, '040055': None,
                           '040056': None, '040057': None, '040058': None, '041009': None, '041010': None,
                           '041011': None, '041012': None, '041013': None, '041014': None, '041015': None,
                           '041016': None, '041042': None, '041043': None, '041044': None, '041045': None,
                           '041046': None, '041047': None, '041048': None, '041049': None, '041050': None,
                           '041051': None, '041052': None, '041053': None, '041054': None, '041055': None,
                           '041056': None, '041057': None, '041058': None, '042009': None, '042010': None,
                           '042011': None, '042012': None, '042013': None, '042014': None, '042015': None,
                           '042016': None, '042043': None, '042044': None, '042045': None, '042046': None,
                           '042047': None, '042048': None, '042049': None, '042050': None, '042051': None,
                           '042052': None, '042053': None, '042054': None, '042055': None, '042056': None,
                           '042057': None, '042058': None, '043008': None, '043009': None, '043010': None,
                           '043011': None, '043012': None, '043013': None, '043014': None, '043015': None,
                           '043044': None, '043045': None, '043046': None, '043047': None, '043048': None,
                           '043049': None, '043050': None, '043051': None, '043052': None, '043053': None,
                           '043054': None, '043055': None, '043056': None, '043057': None, '043058': None,
                           '044009': None, '044010': None, '044011': None, '044012': None, '044013': None,
                           '044014': None, '044015': None, '044044': None, '044045': None, '044046': None,
                           '044047': None, '044048': None, '044049': None, '044050': None, '044051': None,
                           '044052': None, '044053': None, '044054': None, '044055': None, '044056': None,
                           '044057': None, '044058': None, '045008': None, '045009': None, '045010': None,
                           '045011': None, '045012': None, '045013': None, '045014': None, '045015': None,
                           '045044': None, '045045': None, '045046': None, '045047': None, '045048': None,
                           '045049': None, '045050': None, '045051': None, '045052': None, '045053': None,
                           '045054': None, '045055': None, '045056': None, '045057': None, '045058': None,
                           '045059': None, '046008': None, '046009': None, '046010': None, '046011': None,
                           '046012': None, '046013': None, '046014': None, '046015': None, '046045': None,
                           '046046': None, '046047': None, '046048': None, '046049': None, '046050': None,
                           '046051': None, '046052': None, '046053': None, '046054': None, '046055': None,
                           '046056': None, '046057': None, '046058': None, '046059': None, '047008': None,
                           '047009': None, '047010': None, '047011': None, '047012': None, '047013': None,
                           '047014': None, '047015': None, '047046': None, '047047': None, '047048': None,
                           '047049': None, '047050': None, '047051': None, '047052': None, '047053': None,
                           '047054': None, '047055': None, '047056': None, '047057': None, '047058': None,
                           '047059': None, '048008': None, '048009': None, '048010': None, '048011': None,
                           '048012': None, '048013': None, '048014': None, '048015': None, '048046': None,
                           '048047': None, '048048': None, '048049': None, '048050': None, '048051': None,
                           '048052': None, '048053': None, '048054': None, '048055': None, '048056': None,
                           '048057': None, '048058': None, '048059': None, '049008': None, '049009': None,
                           '049010': None, '049011': None, '049012': None, '049013': None, '049014': None,
                           '049015': None, '049046': None, '049047': None, '049048': None, '049049': None,
                           '049050': None, '049051': None, '049052': None, '049053': None, '049054': None,
                           '049055': None, '049056': None, '049057': None, '049058': None, '049059': None,
                           '050007': None, '050008': None, '050009': None, '050010': None, '050011': None,
                           '050012': None, '050013': None, '050014': None, '050015': None, '050016': None,
                           '050047': None, '050048': None, '050049': None, '050050': None, '050051': None,
                           '050052': None, '050053': None, '050054': None, '050055': None, '050056': None,
                           '050057': None, '050058': None, '050059': None, '051008': None, '051009': None,
                           '051010': None, '051011': None, '051012': None, '051013': None, '051014': None,
                           '051015': None, '051048': None, '051049': None, '051050': None, '051051': None,
                           '051052': None, '051053': None, '051054': None, '051055': None, '051056': None,
                           '051057': None, '051058': None, '051059': None, '052007': None, '052008': None,
                           '052009': None, '052010': None, '052011': None, '052012': None, '052013': None,
                           '052014': None, '052048': None, '052049': None, '052050': None, '052051': None,
                           '052052': None, '052053': None, '052054': None, '052055': None, '052056': None,
                           '052057': None, '052058': None, '052059': None, '053008': None, '053009': None,
                           '053010': None, '053011': None, '053012': None, '053013': None, '053049': None,
                           '053050': None, '053051': None, '053052': None, '053053': None, '053054': None,
                           '053055': None, '053056': None, '053057': None, '053058': None, '053059': None,
                           '054007': None, '054008': None, '054009': None, '054010': None, '054011': None,
                           '054012': None, '054013': None, '054014': None, '054015': None, '054049': None,
                           '054050': None, '054051': None, '054052': None, '054053': None, '054054': None,
                           '054055': None, '054056': None, '054057': None, '054058': None, '054059': None,
                           '054060': None, '055007': None, '055008': None, '055009': None, '055010': None,
                           '055011': None, '055012': None, '055013': None, '055014': None, '055015': None,
                           '055042': None, '055043': None, '055050': None, '055051': None, '055052': None,
                           '055053': None, '055054': None, '055055': None, '055056': None, '055057': None,
                           '055058': None, '055059': None, '056007': None, '056008': None, '056009': None,
                           '056010': None, '056011': None, '056012': None, '056013': None, '056014': None,
                           '056043': None, '056044': None, '056051': None, '056052': None, '056053': None,
                           '056054': None, '056055': None, '056056': None, '056057': None, '056058': None,
                           '056059': None, '056060': None, '057007': None, '057008': None, '057009': None,
                           '057010': None, '057011': None, '057012': None, '057013': None, '057014': None,
                           '057015': None, '057044': None, '057045': None, '057050': None, '057051': None,
                           '057052': None, '057053': None, '057054': None, '057055': None, '057056': None,
                           '057057': None, '057058': None, '057059': None, '058007': None, '058008': None,
                           '058009': None, '058010': None, '058011': None, '058012': None, '058013': None,
                           '058014': None, '058046': None, '058051': None, '058052': None, '058053': None,
                           '058054': None, '058055': None, '058056': None, '058057': None, '058058': None,
                           '058059': None, '058060': None, '059007': None, '059008': None, '059009': None,
                           '059010': None, '059013': None, '059014': None, '059046': None, '059050': None,
                           '059051': None, '059052': None, '059053': None, '059054': None, '059055': None,
                           '059056': None, '059057': None, '059058': None, '059059': None, '060007': None,
                           '060008': None, '060009': None, '060010': None, '060013': None, '060014': None,
                           '060047': None, '060049': None, '060050': None, '060051': None, '060052': None,
                           '060053': None, '060054': None, '060055': None, '060056': None, '060057': None,
                           '060058': None, '060059': None, '060060': None, '061008': None, '061009': None,
                           '061010': None, '061012': None, '061013': None, '061014': None, '061046': None,
                           '061047': None, '061048': None, '061049': None, '061050': None, '061051': None,
                           '061052': None, '061053': None, '061054': None, '061055': None, '061056': None,
                           '061057': None, '061058': None, '061059': None, '062007': None, '062008': None,
                           '062009': None, '062010': None, '062011': None, '062012': None, '062013': None,
                           '062014': None, '062046': None, '062047': None, '062048': None, '062049': None,
                           '062050': None, '062051': None, '062052': None, '062053': None, '062054': None,
                           '062055': None, '062056': None, '062057': None, '062058': None, '062059': None,
                           '062060': None, '063008': None, '063009': None, '063010': None, '063011': None,
                           '063012': None, '063013': None, '063014': None, '063046': None, '063047': None,
                           '063048': None, '063049': None, '063050': None, '063051': None, '063052': None,
                           '063053': None, '063054': None, '063055': None, '063056': None, '063057': None,
                           '063058': None, '063059': None, '064007': None, '064008': None, '064009': None,
                           '064010': None, '064011': None, '064012': None, '064013': None, '064014': None,
                           '064047': None, '064048': None, '064049': None, '064050': None, '064051': None,
                           '064052': None, '064053': None, '064054': None, '064055': None, '064056': None,
                           '064057': None, '064058': None, '064059': None, '065008': None, '065009': None,
                           '065010': None, '065011': None, '065012': None, '065013': None, '065047': None,
                           '065048': None, '065049': None, '065050': None, '065051': None, '065052': None,
                           '065053': None, '065054': None, '065055': None, '065056': None, '065057': None,
                           '065058': None, '065059': None, '066008': None, '066009': None, '066010': None,
                           '066011': None, '066012': None, '066013': None, '066014': None, '066047': None,
                           '066048': None, '066049': None, '066050': None, '066051': None, '066052': None,
                           '066053': None, '066054': None, '066055': None, '066056': None, '066057': None,
                           '066058': None, '066059': None, '067008': None, '067009': None, '067010': None,
                           '067011': None, '067012': None, '067013': None, '067046': None, '067047': None,
                           '067048': None, '067049': None, '067050': None, '067051': None, '067052': None,
                           '067053': None, '067054': None, '067055': None, '067056': None, '067057': None,
                           '067058': None, '067059': None, '068008': None, '068009': None, '068010': None,
                           '068011': None, '068012': None, '068013': None, '068046': None, '068047': None,
                           '068048': None, '068049': None, '068050': None, '068051': None, '068052': None,
                           '068053': None, '068054': None, '068055': None, '068056': None, '068057': None,
                           '068058': None, '068059': None, '069008': None, '069009': None, '069010': None,
                           '069011': None, '069012': None, '069013': None, '069046': None, '069047': None,
                           '069048': None, '069049': None, '069050': None, '069051': None, '069052': None,
                           '069053': None, '069054': None, '069055': None, '069056': None, '069057': None,
                           '069058': None, '069059': None, '070008': None, '070009': None, '070010': None,
                           '070011': None, '070012': None, '070013': None, '070014': None, '070045': None,
                           '070046': None, '070047': None, '070048': None, '070049': None, '070050': None,
                           '070051': None, '070052': None, '070053': None, '070054': None, '070055': None,
                           '070056': None, '070057': None, '070058': None, '071008': None, '071009': None,
                           '071010': None, '071011': None, '071012': None, '071013': None, '071014': None,
                           '071015': None, '071044': None, '071045': None, '071046': None, '071047': None,
                           '071048': None, '071049': None, '071050': None, '071051': None, '071052': None,
                           '071053': None, '071054': None, '071055': None, '071056': None, '071057': None,
                           '071058': None, '072009': None, '072010': None, '072011': None, '072012': None,
                           '072013': None, '072014': None, '072015': None, '072044': None, '072045': None,
                           '072046': None, '072047': None, '072048': None, '072049': None, '072050': None,
                           '072051': None, '072052': None, '072053': None, '072054': None, '072055': None,
                           '072056': None, '072057': None, '072058': None, '073009': None, '073010': None,
                           '073011': None, '073012': None, '073013': None, '073014': None, '073015': None,
                           '073044': None, '073045': None, '073046': None, '073047': None, '073048': None,
                           '073049': None, '073050': None, '073051': None, '073052': None, '073053': None,
                           '073054': None, '073055': None, '073056': None, '073057': None, '073058': None,
                           '074009': None, '074010': None, '074011': None, '074012': None, '074013': None,
                           '074014': None, '074015': None, '074043': None, '074044': None, '074045': None,
                           '074046': None, '074047': None, '074048': None, '074049': None, '074050': None,
                           '074051': None, '074052': None, '074053': None, '074054': None, '074055': None,
                           '074056': None, '074057': None, '074058': None, '075010': None, '075011': None,
                           '075012': None, '075013': None, '075014': None, '075015': None, '075043': None,
                           '075044': None, '075045': None, '075046': None, '075047': None, '075048': None,
                           '075049': None, '075050': None, '075051': None, '075052': None, '075053': None,
                           '075054': None, '075055': None, '075056': None, '075057': None, '076009': None,
                           '076010': None, '076011': None, '076012': None, '076013': None, '076014': None,
                           '076015': None, '076043': None, '076044': None, '076045': None, '076046': None,
                           '076047': None, '076048': None, '076049': None, '076050': None, '076051': None,
                           '076052': None, '076053': None, '076054': None, '076055': None, '076056': None,
                           '076057': None, '077010': None, '077011': None, '077012': None, '077013': None,
                           '077014': None, '077015': None, '077042': None, '077043': None, '077044': None,
                           '077045': None, '077046': None, '077047': None, '077048': None, '077049': None,
                           '077050': None, '077051': None, '077052': None, '077053': None, '077054': None,
                           '077055': None, '077056': None, '077057': None, '078010': None, '078011': None,
                           '078012': None, '078013': None, '078014': None, '078015': None, '078016': None,
                           '078017': None, '078018': None, '078043': None, '078044': None, '078045': None,
                           '078046': None, '078047': None, '078048': None, '078049': None, '078050': None,
                           '078051': None, '078052': None, '078053': None, '078054': None, '078055': None,
                           '078056': None, '079010': None, '079011': None, '079012': None, '079013': None,
                           '079014': None, '079015': None, '079016': None, '079017': None, '079018': None,
                           '079019': None, '079042': None, '079043': None, '079044': None, '079045': None,
                           '079046': None, '079047': None, '079048': None, '079049': None, '079050': None,
                           '079051': None, '079052': None, '079053': None, '079054': None, '079055': None,
                           '079056': None, '079057': None, '080011': None, '080012': None, '080013': None,
                           '080014': None, '080015': None, '080016': None, '080017': None, '080018': None,
                           '080019': None, '080020': None, '080025': None, '080026': None, '080042': None,
                           '080043': None, '080044': None, '080045': None, '080046': None, '080047': None,
                           '080048': None, '080049': None, '080050': None, '080051': None, '080052': None,
                           '080053': None, '080054': None, '080055': None, '080056': None, '081011': None,
                           '081012': None, '081013': None, '081014': None, '081015': None, '081016': None,
                           '081017': None, '081018': None, '081019': None, '081020': None, '081021': None,
                           '081022': None, '081023': None, '081024': None, '081025': None, '081026': None,
                           '081041': None, '081042': None, '081043': None, '081044': None, '081045': None,
                           '081046': None, '081047': None, '081048': None, '081049': None, '081050': None,
                           '081051': None, '081052': None, '081053': None, '081054': None, '081055': None,
                           '081056': None, '082011': None, '082012': None, '082013': None, '082014': None,
                           '082015': None, '082016': None, '082017': None, '082018': None, '082019': None,
                           '082020': None, '082021': None, '082022': None, '082023': None, '082024': None,
                           '082026': None, '082027': None, '082041': None, '082042': None, '082043': None,
                           '082044': None, '082045': None, '082046': None, '082047': None, '082048': None,
                           '082049': None, '082050': None, '082051': None, '082052': None, '082053': None,
                           '082054': None, '082055': None, '082056': None, '083012': None, '083013': None,
                           '083014': None, '083015': None, '083016': None, '083017': None, '083018': None,
                           '083019': None, '083020': None, '083021': None, '083022': None, '083023': None,
                           '083024': None, '083040': None, '083041': None, '083043': None, '083044': None,
                           '083045': None, '083046': None, '083047': None, '083048': None, '083049': None,
                           '083050': None, '083051': None, '083052': None, '083053': None, '083054': None,
                           '083055': None, '084012': None, '084013': None, '084014': None, '084015': None,
                           '084016': None, '084017': None, '084018': None, '084019': None, '084020': None,
                           '084021': None, '084022': None, '084023': None, '084024': None, '084027': None,
                           '084028': None, '084039': None, '084040': None, '084043': None, '084044': None,
                           '084045': None, '084046': None, '084047': None, '084048': None, '084049': None,
                           '084050': None, '084051': None, '084052': None, '084053': None, '084054': None,
                           '085012': None, '085013': None, '085014': None, '085017': None, '085018': None,
                           '085019': None, '085020': None, '085021': None, '085022': None, '085023': None,
                           '085024': None, '085025': None, '085026': None, '085027': None, '085028': None,
                           '085029': None, '085039': None, '085040': None, '085041': None, '085043': None,
                           '085044': None, '085045': None, '085046': None, '085047': None, '085048': None,
                           '085049': None, '085050': None, '085051': None, '085052': None, '085053': None,
                           '085054': None, '086013': None, '086014': None, '086017': None, '086018': None,
                           '086019': None, '086020': None, '086021': None, '086022': None, '086023': None,
                           '086024': None, '086025': None, '086026': None, '086027': None, '086028': None,
                           '086029': None, '086030': None, '086039': None, '086040': None, '086041': None,
                           '086042': None, '086043': None, '086044': None, '086045': None, '086046': None,
                           '086047': None, '086048': None, '086049': None, '086050': None, '086051': None,
                           '086052': None, '086053': None, '086054': None, '087017': None, '087018': None,
                           '087019': None, '087020': None, '087021': None, '087022': None, '087023': None,
                           '087024': None, '087025': None, '087026': None, '087027': None, '087028': None,
                           '087029': None, '087030': None, '087031': None, '087032': None, '087033': None,
                           '087039': None, '087040': None, '087041': None, '087042': None, '087043': None,
                           '087044': None, '087045': None, '087046': None, '087047': None, '087048': None,
                           '087049': None, '087050': None, '087051': None, '087052': None, '087053': None,
                           '088014': None, '088015': None, '088017': None, '088018': None, '088019': None,
                           '088020': None, '088021': None, '088022': None, '088023': None, '088024': None,
                           '088025': None, '088026': None, '088027': None, '088028': None, '088029': None,
                           '088030': None, '088031': None, '088032': None, '088033': None, '088034': None,
                           '088035': None, '088039': None, '088040': None, '088041': None, '088042': None,
                           '088043': None, '088044': None, '088045': None, '088046': None, '088047': None,
                           '088048': None, '088049': None, '088050': None, '088051': None, '088052': None,
                           '088053': None, '089014': None, '089015': None, '089016': None, '089017': None,
                           '089018': None, '089019': None, '089020': None, '089021': None, '089022': None,
                           '089023': None, '089024': None, '089025': None, '089026': None, '089027': None,
                           '089028': None, '089029': None, '089030': None, '089031': None, '089032': None,
                           '089033': None, '089034': None, '089035': None, '089039': None, '089040': None,
                           '089041': None, '089042': None, '089043': None, '089044': None, '089045': None,
                           '089046': None, '089047': None, '089050': None, '089051': None, '089052': None,
                           '090015': None, '090016': None, '090017': None, '090018': None, '090020': None,
                           '090021': None, '090022': None, '090023': None, '090025': None, '090026': None,
                           '090027': None, '090028': None, '090029': None, '090031': None, '090032': None,
                           '090033': None, '090034': None, '090035': None, '090036': None, '090039': None,
                           '090040': None, '090041': None, '090042': None, '090043': None, '090044': None,
                           '090045': None, '090046': None, '090047': None, '090050': None, '090051': None,
                           '090052': None, '091016': None, '091017': None, '091020': None, '091021': None,
                           '091022': None, '091023': None, '091026': None, '091027': None, '091030': None,
                           '091031': None, '091032': None, '091033': None, '091034': None, '091035': None,
                           '091036': None, '091038': None, '091039': None, '091040': None, '091041': None,
                           '091042': None, '091043': None, '091044': None, '091045': None, '091046': None,
                           '091047': None, '091050': None, '091051': None, '092017': None, '092021': None,
                           '092022': None, '092028': None, '092029': None, '092030': None, '092031': None,
                           '092032': None, '092033': None, '092034': None, '092035': None, '092036': None,
                           '092037': None, '092038': None, '092039': None, '092040': None, '092043': None,
                           '092044': None, '092045': None, '092046': None, '092047': None, '092049': None,
                           '092050': None, '093017': None, '093020': None, '093021': None, '093022': None,
                           '093023': None, '093024': None, '093028': None, '093029': None, '093030': None,
                           '093031': None, '093032': None, '093033': None, '093034': None, '093035': None,
                           '093036': None, '093037': None, '093038': None, '093039': None, '093043': None,
                           '093044': None, '093045': None, '093046': None, '093047': None, '093049': None,
                           '093050': None, '094018': None, '094019': None, '094020': None, '094021': None,
                           '094022': None, '094023': None, '094024': None, '094025': None, '094026': None,
                           '094027': None, '094028': None, '094029': None, '094030': None, '094031': None,
                           '094032': None, '094033': None, '094034': None, '094035': None, '094036': None,
                           '094037': None, '094038': None, '094039': None, '094043': None, '094044': None,
                           '094045': None, '094046': None, '094047': None, '094048': None, '094049': None,
                           '095018': None, '095019': None, '095020': None, '095021': None, '095022': None,
                           '095023': None, '095024': None, '095025': None, '095026': None, '095027': None,
                           '095028': None, '095029': None, '095030': None, '095031': None, '095032': None,
                           '095033': None, '095034': None, '095035': None, '095036': None, '095037': None,
                           '095038': None, '095039': None, '095043': None, '095044': None, '095045': None,
                           '095046': None, '095047': None, '095048': None, '096019': None, '096020': None,
                           '096021': None, '096022': None, '096023': None, '096024': None, '096025': None,
                           '096026': None, '096027': None, '096029': None, '096030': None, '096031': None,
                           '096032': None, '096033': None, '096034': None, '096035': None, '096036': None,
                           '096037': None, '096038': None, '096039': None, '096040': None, '096043': None,
                           '096044': None, '096045': None, '096046': None, '096047': None, '097020': None,
                           '097021': None, '097022': None, '097023': None, '097024': None, '097025': None,
                           '097026': None, '097027': None, '097028': None, '097029': None, '097030': None,
                           '097031': None, '097032': None, '097033': None, '097034': None, '097035': None,
                           '097036': None, '097037': None, '097038': None, '097039': None, '097040': None,
                           '097044': None, '097045': None, '097046': None, '097047': None, '098021': None,
                           '098022': None, '098023': None, '098024': None, '098025': None, '098026': None,
                           '098027': None, '098028': None, '098029': None, '098030': None, '098031': None,
                           '098032': None, '098033': None, '098034': None, '098035': None, '098036': None,
                           '098037': None, '098038': None, '098039': None, '098040': None, '098041': None,
                           '098043': None, '098044': None, '098045': None, '098046': None, '099022': None,
                           '099023': None, '099024': None, '099025': None, '099026': None, '099027': None,
                           '099028': None, '099029': None, '099030': None, '099031': None, '099032': None,
                           '099033': None, '099034': None, '099035': None, '099036': None, '099037': None,
                           '099038': None, '099039': None, '099040': None, '099041': None, '099042': None,
                           '099043': None, '099044': None, '100024': None, '100025': None, '100026': None,
                           '100027': None, '100028': None, '100029': None, '100030': None, '100031': None,
                           '100032': None, '100033': None, '100034': None, '100035': None, '100036': None,
                           '100037': None, '100038': None, '100039': None, '100040': None, '100041': None,
                           '100042': None, '100043': None, '101026': None, '101027': None, '101028': None,
                           '101029': None, '101030': None, '101031': None, '101032': None, '101033': None,
                           '101034': None, '101035': None, '101036': None, '101037': None, '101038': None,
                           '101039': None, '101040': None, '101041': None, '102027': None, '102028': None,
                           '102029': None, '102030': None, '102031': None, '102032': None, '102033': None,
                           '102034': None, '102035': None, '102036': None, '102037': None, '102038': None,
                           '102039': None, '103031': None, '103032': None, '103033': None, '103034': None,
                           '103035': None, '103036': None}

def is_in_unconstrained(node):
    if node in unconstrained_node_dict:
        return True
    else:
        return False
