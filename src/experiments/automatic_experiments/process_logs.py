import json


def open_file(path):
    f = open(path, 'r')
    data2 = json.load(f)
    return data2

def parse_matching(matching_data):
    return eval(matching_data)

def parse(data):
    expariment_logs = []
    robot_types = {}
    for step in data["logs"]:
        matching = parse_matching(step["matching"])
        expariment_logs.append({
            "matching": matching,
            "cost": step["cost"],
            "tick": step["tick"]
        })
    for type in data["robot_types"]:
        if type["type"] not in robot_types:
            robot_types[type["type"]] = []
        robot_types[type["type"]].append(int(type["robot_id"]))
        
    return robot_types, expariment_logs

def how_many_times_matching_changed(logs):
    count = 0
    for i in range(1,len(logs)):
        if(logs[i]["matching"] != logs[i-1]["matching"]):
            count += 1
    return count

def get_nf_pairs(matching, robot_types):
    nf_pairs = []
    for pair in matching:
        if pair[0] in robot_types["CRobustMatching"] and pair[1] in robot_types["CRobustMatching"]:
            nf_pairs.append(pair)
    return nf_pairs


if __name__ == "__main__":
    a = open_file("/Users/lior.strichash/private/robust-matching/src/experiments/random/5_faults.argos.log")
    robot_types, logs = parse(a)
    stablity = how_many_times_matching_changed(logs)
    c = get_nf_pairs(logs[1000]["matching"], robot_types)
    print(c)