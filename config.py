# Configs
dt_sim = 0.001 # Dynamics update rate
dt_viewer = 0.02 # Viewer update rate
robot_type = "biped" # "hopper" or "biped"
robot_state_topic = robot_type + "_state"
robot_cmd_topic = robot_type + "_control"
if robot_type == "hopper":
    robot_xml_path = "./robot_assets/Hopper/hopper_scene.xml"
elif robot_type == "biped":
    robot_xml_path = "./robot_assets/BipedLinefoot/biped_linefoot_scene.xml"
