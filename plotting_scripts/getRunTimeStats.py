import argparse

def extract_time_info(file_path):
    num_all_markers = None
    simulation_time = None
    end_time = None

    with open(file_path, 'r') as file:
        for line in file:
            if "numAllMarkers" in line:
                num_all_markers = int(line.split(':')[-1].strip())
            elif "Simulation time" in line:
                simulation_time = float(line.split(':')[-1].strip().split()[0])
            elif "End Time" in line:
                end_time = float(line.split(':')[-1].strip().split()[0])
    
    return num_all_markers, simulation_time, end_time



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=str, help="Path to the bin feature FSI")

    args = parser.parse_args()


    demo_kinds = ["BaffleFlow", "DamBreak", "FlexibleCable", "FlexibleFlatPlate", "SphereBounce", "WaveTank"]
    for demo in demo_kinds:
        fastPS_folder = "../" + demo + "Performance/"

        if args.base.endswith('/'):
            args.base = args.base[:-1]
        
        
        feature_fsi_folder = args.base + "/" + demo + "Performance/"

        # =================
        # FastPS
        # =================
        ns_options = ["1" , "2", "4", "8"]
        if((demo == "BaffleFlow")):
            for ns in ns_options:
                for scale in ["1", "2", "3"]:
                    file_path = fastPS_folder + ns + "_" + scale + ".log"
                    num_all_markers, simulation_time, end_time = extract_time_info(file_path)
                    timePerSec = simulation_time / end_time
                    print(f"{demo} & {ns} & {num_all_markers} & {simulation_time} & {end_time} & {timePerSec} \\\\")
                    print("\\hline")
        else:
            for ns in ns_options:
                file_path = fastPS_folder + ns + ".log"
                num_all_markers, simulation_time, end_time = extract_time_info(file_path)
                timePerSec = simulation_time / end_time
                print(f"{demo} & {ns} & {num_all_markers} & {simulation_time} & {end_time} & {timePerSec} \\\\")
                print("\\hline")

        

        # =================
        # FeatureFSI
        # =================
        if((demo == "BaffleFlow")):
            for scale in ["1", "2", "3"]:
                file_path = feature_fsi_folder + scale + ".log"
                num_all_markers, simulation_time, end_time = extract_time_info(file_path)
                timePerSec = simulation_time / end_time
                print(f"Old Code : {demo} & {num_all_markers} & {simulation_time} & {end_time} & {timePerSec} \\\\")
                print("\\hline")
        else:
            file_path = feature_fsi_folder + "1.log"
            num_all_markers, simulation_time, end_time = extract_time_info(file_path)
            timePerSec = simulation_time / end_time
            
            print(f"Old Code : {demo} & 1 & {num_all_markers} & {simulation_time} & {end_time} & {timePerSec} \\\\")
            print("\\hline")

        






