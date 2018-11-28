import yaml
import csv
import utm

if __name__ == "__main__":
    f = open("coastline.yaml", "r+")
    csv_f = open('coastline.csv', 'w')
    data = yaml.load(f)
    nodes = data['node'].keys()
    keys = data['way'].keys()
    writer = csv.writer(csv_f, lineterminator='\n')
    for key in keys:
        value = data['way'][key]
        start_node_id = value['start_node_id']
        end_node_id = value['end_node_id']
        start_node_lat = data['node'][start_node_id]['lat']
        start_node_lon = data['node'][start_node_id]['lon']
        end_node_lat = data['node'][end_node_id]['lat']
        end_node_lon = data['node'][end_node_id]['lon']
        start_utm_pos = utm.from_latlon(start_node_lat,start_node_lon)
        end_utm_pos = utm.from_latlon(end_node_lat,end_node_lon)
        writer.writerow(start_utm_pos + end_utm_pos)
    f.close
    csv_f.close