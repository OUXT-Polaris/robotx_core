import overpy
import yaml

def fetch_result(lat,lon):
    size = 0.02
    api = overpy.Overpass()
    coastline_request = "way(" + str(lat-size/2) + "," + str(lon-size/2) + "," + str(lat+size/2) + "," + str(lon+size/2) + "[coastline]);out;"
    coastline_result = api.query(coastline_request)
    node_request = "node(" + str(lat-size/2) + "," + str(lon-size/2) + "," + str(lat+size/2) + "," + str(lon+size/2) + ");out;"
    node_result = api.query(node_request)
    coastline_data = {}
    coastline_way_data = {}
    coastline_node_data = {}
    f = open("coastline.yaml", "w")
    for coastline_node in node_result.nodes:
        node_data = {}
        node_data["lat"] = float(coastline_node.lat)
        node_data["lon"] = float(coastline_node.lon)
        coastline_node_data[str(coastline_node.id)] = node_data
    way_id = 0
    for coastline_way in coastline_result.ways:
        if coastline_way.tags.get("natural") == u'coastline':
            for i in range(len(coastline_way._node_ids)-1):
                if str(coastline_way._node_ids[i]) in coastline_node_data:
                    if str(coastline_way._node_ids[i+1]) in coastline_node_data:
                        way_data = {}
                        way_data["start_node_id"] = str(coastline_way._node_ids[i])
                        way_data["end_node_id"] = str(coastline_way._node_ids[i+1])
                        way_id = way_id + 1
                        coastline_way_data[str(way_id)] = way_data
    coastline_data["way"] = coastline_way_data
    coastline_data["node"] = coastline_node_data
    f.write(yaml.dump(coastline_data))
    f.close()
    
if __name__ == '__main__':
    fetch_result(21.305,-157.89)