import subprocess, os
"""

Opens the files that end with .raw
feeds these filese to to the C binary to convert them to CSV
deletes the .raw files

"""

path = "/Users/efesaatci/Desktop/projects/habitslab/day_exp/P0/Wild/Camera/Raw"
fun = lambda x: x.endswith("raw")
raw_files = filter(fun, os.listdir(path))
raw_files = map(lambda x: os.path.join(path, x), raw_files)

for rf in raw_files:
    name = os.path.splitext(rf)[0]        
    csv_file = "{}.CSV".format(name)
    csv_path = os.path.join(path, csv_file)
    with open(rf, "r") as f:
        with open(csv_path, "w") as cf:
            list_files = subprocess.Popen(["./a.out"], stdin=f, stdout=cf)
    
    os.remove(rf)
    

