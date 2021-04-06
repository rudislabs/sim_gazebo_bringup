from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from time import sleep
import json
import numpy as np
import shlex
import sys
import subprocess

# Set relative paths to real paths
launch_path = os.path.realpath(__file__).replace("sim_gazebo.launch.py","")
json_path = os.path.realpath(os.path.relpath(os.path.join(launch_path,"../config")))
ros2_ws = os.path.realpath(os.path.relpath(os.path.join(launch_path,"../../..")))

# Set useful variables
default_controller="px4"
gzclient_verbose='false'
gzserver_verbose='false'

# Initialize flags
has_reset_gazebo_model_env=False
has_reset_gazebo_plugin_env=False
has_reset_gazebo_resource_env=False
has_reset_ld_library_env=True
has_reset_sitl_gazebo_path_env=False
has_built_ros2_pkgs=False
clean_start=True
debug_verbose=False
overide_gazebo=False

# Clear /tmp output from previous runs
if clean_start:
    print("INFO: Cleaning previous temporary models")
    os.system("pkill -9 xterm")
    os.system("pkill -9 gzclient")
    os.system("pkill -9 gzserver")
    os.system("pkill -9 px4*")
    os.system("rm -rf /tmp/sitl*")
    os.system("rm -rf /tmp/gazebo")
    os.system("rm -rf /tmp/px4*")

# Open JSON configuration file
with open('{:s}/gen_params.json'.format(json_path)) as json_file:
    json_params = json.load(json_file)

# Setup related configs
setup_autopilot = json_params["setup"]["autopilot"]
setup_gazebo = json_params["setup"]["gazebo"]
setup_system = json_params["setup"]["system"]
setup_ros2 = json_params["setup"]["ros2"]


# Runtime related params
ros2_nodes= json_params["nodes"]
world_params = json_params["world_params"]
models = json_params["models"]

# Set verbose mode if present
if "verbose" in json_params:
    if "gzclient" in json_params["verbose"]:
        gzclient_verbose = str(json_params["verbose"]["gzclient"]).lower()
    if "gzserver" in json_params["verbose"]:
        gzserver_verbose = str(json_params["verbose"]["gzserver"]).lower()

########################################################################################

# Iterate through defined Gazebo model repos
for repo in setup_gazebo["gazebo_models"]:
    gazebo_repo = setup_gazebo["gazebo_models"][repo]
    gazebo_repo_path = '{:s}/{:s}'.format(ros2_ws, 
        gazebo_repo["name"])

    # Clone Gazebo model repo if not present
    if not os.path.isdir(gazebo_repo_path):
        clone_cmd = 'git clone -b {:s} {:s} {:s}'.format(
            gazebo_repo["version"],gazebo_repo["repo"], gazebo_repo_path)
        clone_cmd_popen=shlex.split(clone_cmd)
        clone_popen = subprocess.Popen(clone_cmd_popen, 
            stdout=subprocess.PIPE, text=True)
        while True:
            output = clone_popen.stdout.readline()
            if output == '' and clone_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        clone_popen.wait()

    # Handle complicated model paths
    if gazebo_repo_path.endswith("/models"):
        gazebo_repo_path = str(os.path.realpath(os.path.join(gazebo_repo_path,'..')))
        if os.path.isdir(os.path.join(gazebo_repo_path,'models/.git')):
            os.system('rm -rf {:s}/models/.git'.format(gazebo_repo_path))


    # Check to see if Gazebo model path environment variable is reset yet, if not reset to avoid other models
    if not has_reset_gazebo_model_env:
        os.environ['GAZEBO_MODEL_PATH'] = '{:s}/models:/tmp/gazebo/models'.format(
            gazebo_repo_path)
        has_reset_gazebo_model_env=True
    
    # Append to Gazebo model path environment for subsequent repos if not present
    if has_reset_gazebo_model_env and ('{:s}/models'.format(gazebo_repo_path) not in os.getenv('GAZEBO_MODEL_PATH')):
        os.environ['GAZEBO_MODEL_PATH'] = '{:s}/models:{:s}'.format(
            gazebo_repo_path, os.getenv('GAZEBO_MODEL_PATH'))

    # Check to see if Gazebo model path environment variable is reset yet, if not reset to avoid other models
    if not has_reset_gazebo_resource_env:
        os.environ['GAZEBO_RESOURCE_PATH'] = '{:s}:/tmp/gazebo'.format(
            gazebo_repo_path)
        has_reset_gazebo_resource_env=True
    
    # Append to Gazebo model path environment for subsequent repos if not present
    if has_reset_gazebo_resource_env and ('{:s}'.format(gazebo_repo_path) not in os.getenv('GAZEBO_RESOURCE_PATH')):
        os.environ['GAZEBO_RESOURCE_PATH'] = '{:s}:{:s}'.format(
            gazebo_repo_path, os.getenv('GAZEBO_RESOURCE_PATH'))

########################################################################################

# Iterate through defined autopilot repos
for build in setup_autopilot:
    autopilot_build = setup_autopilot[build]
    autopilot_path = '{:s}/{:s}'.format(ros2_ws, 
        autopilot_build["name"])
    autopilot_build_path = '{:s}/build/{:s}'.format(autopilot_path, 
        autopilot_build["build_type"])

    # Clone autopilot repo if not present
    if (not os.path.isdir(autopilot_path)) and (autopilot_build["clone"]):
        clone_cmd = 'git clone -b {:s} {:s} {:s}'.format(
            autopilot_build["version"],autopilot_build["repo"], autopilot_path)
        clone_cmd_popen=shlex.split(clone_cmd)
        clone_popen = subprocess.Popen(clone_cmd_popen, 
            stdout=subprocess.PIPE, text=True)
        while True:
            output = clone_popen.stdout.readline()
            if output == '' and clone_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        clone_popen.wait()

    # Build autopilot if not built or incorrect version built
    if (os.path.isdir(autopilot_path)) and (not os.path.isdir(autopilot_build_path)):
        build_cmd = 'make clean && DONT_RUN=1 make {:s} {:s} {:s}'.format(
            autopilot_build["build_prefix"],autopilot_build["build_type"],
            autopilot_build["build_postfix"])
        build_cmd_popen=shlex.split(build_cmd)
        build_popen = subprocess.Popen(build_cmd_popen, stdout=subprocess.PIPE, 
            cwd=autopilot_path, text=True)
        while True:
            output = build_popen.stdout.readline()
            if output == '' and build_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        build_popen.wait()

    # Add to sitl gazebo path if no sitl gazebo path
    if not has_reset_sitl_gazebo_path_env or os.getenv('SITL_GAZEBO_PATH') is None \
        and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['SITL_GAZEBO_PATH'] = '{:s}/Tools/sitl_gazebo'.format(autopilot_path)
        has_reset_sitl_gazebo_path_env = True

    # Add to sitl gazebo path if not already in sitl gazebo path
    elif '{:s}/Tools/sitl_gazebo'.format(autopilot_path) not in os.getenv('SITL_GAZEBO_PATH') \
        and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['SITL_GAZEBO_PATH'] = '{:s}:{:s}'.format(
            '{:s}/Tools/sitl_gazebo'.format(autopilot_path), 
            os.getenv('SITL_GAZEBO_PATH'))

    # Add to library path if no library path
    if (not has_reset_ld_library_env or os.getenv('LD_LIBRARY_PATH') is None) \
        and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['LD_LIBRARY_PATH'] = '{:s}/build_gazebo'.format(
            autopilot_build_path)
        has_reset_ld_library_env = True

    # Add to library path if not already in library path
    elif ('{:s}/build_gazebo'.format(autopilot_build_path) not in os.getenv('LD_LIBRARY_PATH')) \
        and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['LD_LIBRARY_PATH'] = '{:s}/build_gazebo:{:s}'.format(
            autopilot_build_path, os.getenv('LD_LIBRARY_PATH'))

    # Check to see if Gazebo plugin path environment variable is reset yet, if not reset to avoid plugin issues
    # Only include plugin if set to be used in JSON setup.autopilot.autopilot_build_params__.gazebo_plugins
    if (not has_reset_gazebo_plugin_env and autopilot_build["source_gazebo_plugins"]) \
        and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}/build_gazebo'.format(
            autopilot_build_path)
        has_reset_gazebo_plugin_env=True
    
    # Append to Gazebo plugin path environment for subsequent builds if not present and set to be used
    elif (autopilot_build["source_gazebo_plugins"] and ('{:s}/build_gazebo'.format(autopilot_build_path) 
        not in os.getenv('GAZEBO_PLUGIN_PATH'))) and autopilot_build["build_postfix"] == 'gazebo':
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}/build_gazebo:{:s}'.format(
            autopilot_build_path, os.getenv('GAZEBO_PLUGIN_PATH'))

########################################################################################

# Iterate through defined gazebo_plugins repos
for build in setup_gazebo["gazebo_plugins"]:
    plugin_build = setup_gazebo["gazebo_plugins"][build]
    plugin_path = '{:s}/{:s}'.format(ros2_ws, 
        plugin_build["name"])
    plugin_mavlink_path = '{:s}/{:s}'.format(ros2_ws, 
        plugin_build["workspace_relative_mavlink"])
    plugin_mavlink_path = '{:s}/{:s}'.format(ros2_ws, 
        plugin_build["workspace_relative_mavlink"])
    plugin_build_path = '{:s}/build'.format(plugin_path)

    # Clone plugin repo if not present
    if (not os.path.isdir(plugin_path)):
        clone_cmd = 'git clone --recursive -b {:s} {:s} {:s}'.format(
            plugin_build["version"],plugin_build["repo"], plugin_path)
        clone_cmd_popen=shlex.split(clone_cmd)
        clone_popen = subprocess.Popen(clone_cmd_popen, 
            stdout=subprocess.PIPE, text=True)
        while True:
            output = clone_popen.stdout.readline()
            if output == '' and clone_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        clone_popen.wait()

    # Build plugin if not built
    if (os.path.isdir(plugin_path)) and (not os.path.isdir(plugin_build_path)):
        try: 
            os.makedirs(plugin_build_path, exist_ok = True) 
        except OSError as error: 
            print("Directory creation error.")

        cmake_cmd = 'cmake .. -D_MAVLINK_INCLUDE_DIR={:s}'.format(
            plugin_mavlink_path)
        cmake_cmd_popen=shlex.split(cmake_cmd)
        cmake_popen = subprocess.Popen(cmake_cmd_popen, stdout=subprocess.PIPE, 
            cwd=plugin_build_path, text=True)
        while True:
            output = cmake_popen.stdout.readline()
            if output == '' and cmake_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        cmake_popen.wait()

        make_cmd = 'make -j{:s} -l{:s}'.format(
            str(int(np.floor(os.cpu_count()*.75))), str(int(np.floor(os.cpu_count()*.75))))
        make_cmd_popen=shlex.split(make_cmd)
        make_popen = subprocess.Popen(make_cmd_popen, stdout=subprocess.PIPE, 
            cwd=plugin_build_path, text=True)
        while True:
            output = make_popen.stdout.readline()
            if output == '' and make_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        make_popen.wait()

    # Add to sitl gazebo path if no sitl gazebo path
    if not has_reset_sitl_gazebo_path_env or os.getenv('SITL_GAZEBO_PATH') is None:
        os.environ['SITL_GAZEBO_PATH'] = plugin_path
        has_reset_sitl_gazebo_path_env = True

    # Add to sitl gazebo path if not already in sitl gazebo path
    elif plugin_path not in os.getenv('SITL_GAZEBO_PATH'):
        os.environ['SITL_GAZEBO_PATH'] = '{:s}:{:s}'.format(
            plugin_path, os.getenv('SITL_GAZEBO_PATH'))
    
    # Add to library path if no library path
    if not has_reset_ld_library_env or os.getenv('LD_LIBRARY_PATH') is None:
        os.environ['LD_LIBRARY_PATH'] = '{:s}/build_gazebo'.format(
            autopilot_build_path)
        has_reset_ld_library_env = True

    # Add to library path if not already in library path
    elif '{:s}/build_gazebo'.format(plugin_build_path) not in os.getenv('LD_LIBRARY_PATH'):
        os.environ['LD_LIBRARY_PATH'] = '{:s}:{:s}'.format(
            plugin_build_path, os.getenv('LD_LIBRARY_PATH'))

    # Check to see if Gazebo plugin path environment variable is reset yet, if not reset to avoid plugin issues
    if not has_reset_gazebo_plugin_env and plugin_build["source_gazebo_plugins"]:
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}'.format(
            plugin_build_path)
        has_reset_gazebo_plugin_env=True
    
    # Append to Gazebo plugin path environment for subsequent repos if not present and set to be used
    elif plugin_build["source_gazebo_plugins"] and ('{:s}/build_gazebo'.format(plugin_build_path) 
        not in os.getenv('GAZEBO_PLUGIN_PATH')):
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}:{:s}'.format(
            plugin_build_path, os.getenv('GAZEBO_PLUGIN_PATH'))

########################################################################################

# Set environment variables if present
if "set_environment" in setup_system:
    for env in setup_system["set_environment"]:
        set_env = setup_system["set_environment"][env]
        if set_env["method"] == "overwrite":
            os.environ[set_env["variable"]] = set_env["value"]
        elif set_env["method"] == "prepend":
            if os.getenv(set_env["variable"]) is None:
                os.environ[set_env["variable"]] = set_env["value"]
            else:
                os.environ[set_env["variable"]] = '{:s}:{:s}'.format(
                    set_env["value"], os.getenv(set_env["variable"]))
        elif set_env["method"] == "postpend":
            if os.getenv(set_env["variable"]) is None:
                os.environ[set_env["variable"]] = set_env["value"]
            else:
                os.environ[set_env["variable"]] = '{:s}:{:s}'.format(
                    os.getenv(set_env["variable"]), set_env["value"])

# Always use default if none set
if os.getenv('GAZEBO_PLUGIN_PATH') is None:
    os.environ['GAZEBO_PLUGIN_PATH'] = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
if os.getenv('GAZEBO_RESOURCE_PATH') is None:
    os.environ['GAZEBO_RESOURCE_PATH'] = "/usr/share/gazebo-11"

if debug_verbose:
    print('GAZEBO_PLUGIN_PATH= {:s}'.format(os.getenv('GAZEBO_PLUGIN_PATH')))
    print('GAZEBO_RESOURCE_PATH= {:s}'.format(os.getenv('GAZEBO_RESOURCE_PATH')))
    print('GAZEBO_MODEL_PATH= {:s}'.format(os.getenv('GAZEBO_MODEL_PATH')))
    print('LD_LIBRARY_PATH= {:s}'.format(os.getenv('LD_LIBRARY_PATH')))
    print('SITL_GAZEBO_PATH= {:s}'.format(os.getenv('SITL_GAZEBO_PATH')))

########################################################################################

for build in setup_ros2:
    ros2_pkg_build = setup_ros2[build]
    ros2_pkg_path = '{:s}/src/{:s}'.format(ros2_ws,ros2_pkg_build["build_package"])
    if (not os.path.isdir(ros2_pkg_path)):
        has_built_ros2_pkgs=True
        clone_cmd = 'git clone -b {:s} {:s} {:s}'.format(
            ros2_pkg_build["version"],ros2_pkg_build["repo"], ros2_pkg_path)
        clone_cmd_popen=shlex.split(clone_cmd)
        clone_popen = subprocess.Popen(clone_cmd_popen, 
            stdout=subprocess.PIPE, text=True)
        while True:
            output = clone_popen.stdout.readline()
            if output == '' and clone_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        clone_popen.wait()

        build_cmd = 'colcon build {:s} {:s} {:s}'.format(
            ros2_pkg_build["build_prefix"],ros2_pkg_build["build_package"],
            ros2_pkg_build["build_postfix"])
        build_cmd_popen=shlex.split(build_cmd)
        build_popen = subprocess.Popen(build_cmd_popen, stdout=subprocess.PIPE, 
            cwd=ros2_ws, text=True)
        while True:
            output = build_popen.stdout.readline()
            if output == '' and build_popen.poll() is not None:
                break
            if output:
                print(output.strip())
        build_popen.wait()

# Require restart if colcon packages built so they can be correctly found
if has_built_ros2_pkgs:
    src_ros2_ws = '{:s}/install/setup.bash'.format(ros2_ws)
    os.system('/bin/bash {:s}'.format(src_ros2_ws))
    os.system("/bin/bash /opt/ros/foxy/setup.bash")
    print('''\n\n\nPLEASE RUN:\n 
        source {:s}; source /opt/ros/foxy/setup.bash; ros2 launch sim_gazebo_bringup sim_gazebo.launch.py
        \n\n'''.format(src_ros2_ws))
    sys.exit()

########################################################################################

# Unbind connections if bound, this can be dangerous 
# as it will kill any processes bound to that connection.
if "connections" in setup_system:
    for connection in setup_system["connections"]:
        unbind_connection = setup_system["connections"][connection]
        if unbind_connection["force_unbind"]:
            for port in unbind_connection["ports"]:
                try:
                    os.system('fuser -k {:s}/{:s}'.format(
                        str(port),str(unbind_connection["transport"]).lower()))
                except OSError as error: 
                    print('\n{:s} unbind failed for port: {:s}\n'.format(
                        str(unbind_connection["transport"]).lower(), str(port)))


########################################################################################

# Generate the world
generate_world_params = world_params["generate_params"]
if world_params["generate_world"]:
    generate_world_args=""
    for params in generate_world_params:
        generate_world_args += ' --{:s} "{:s}"'.format(params, 
            str(generate_world_params[params]))

    generate_world_cmd = 'python3 {:s}/{:s}/scripts/jinja_world_gen.py{:s}'.format(
        ros2_ws, world_params["gazebo_name"], generate_world_args
        ).replace("\n","").replace("    ","")
    if debug_verbose:
        print('\nGenerating world with: {:s}\n'.format(generate_world_cmd))
    world_cmd_popen=shlex.split(generate_world_cmd)
    world_popen = subprocess.Popen(world_cmd_popen, stdout=subprocess.PIPE, text=True)
    while True:
        output = world_popen.stdout.readline()
        if output == '' and world_popen.poll() is not None:
            break
        if output:
            print(output.strip())
    world_popen.wait()

    world_file_path='/tmp/gazebo/worlds/{:s}.world'.format(generate_world_params["world_name"])

else:
    world_file_path='{:s}/{:s}/worlds/{:s}.world'.format(ros2_ws,
        world_params["gazebo_name"],generate_world_params["world_name"])

# Set relevant world values
latitude = generate_world_params["lat_lon_alt"][0]
longitude = generate_world_params["lat_lon_alt"][1]
altitude = generate_world_params["lat_lon_alt"][2]

if debug_verbose:
    print('latitude: {:s}, longitude: {:s} altitude: {:s}'.format(
        str(latitude), str(longitude), str(altitude)))

if overide_gazebo:
    gazebo_cmd = 'gazebo --verbose {:s}'.format(str(world_file_path))
    gazebo_cmd_popen=shlex.split(gazebo_cmd)
    gazebo_popen = subprocess.Popen(gazebo_cmd_popen, 
        stdout=subprocess.PIPE, text=True)

########################################################################################

def generate_launch_description():

    ld = LaunchDescription([
        # World path argument
        DeclareLaunchArgument(
            'world_path', default_value= world_file_path,
            description='Provide full world file path and name'),
        DeclareLaunchArgument(
            'gzserver_verbose', default_value= gzserver_verbose,
            description='Run gzserver in verbose mode'),
        DeclareLaunchArgument(
            'gzclient_verbose', default_value= gzclient_verbose,
            description='Run gzclient in verbose mode'),
        ])


    if not overide_gazebo:
        # Get path to gazebo package
        gazebo_package_prefix = get_package_share_directory('gazebo_ros')

        # Launch gazebo servo with world file from world_path
        gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzserver.launch.py']),
                    launch_arguments={
                        'world': LaunchConfiguration('world_path'),
                        'verbose': LaunchConfiguration('gzserver_verbose')
                        }.items(),
                    )

        ld.add_action(gazebo_server)

        # Launch gazebo client
        gazebo_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzclient.launch.py']),
            launch_arguments={
                        'verbose': LaunchConfiguration('gzclient_verbose')
                        }.items(),
            )

        ld.add_action(gazebo_client)

    ####################################################################################

    # pre-spawn ROS2 Nodes
    for node in ros2_nodes:
        ros2_node = ros2_nodes[node]

        # Run if timing set to pre-spawn of model
        if ros2_node["timing"] == "pre-spawn":
            if "remappings" in ros2_node:
                remappings = [eval(str(ros2_node["remappings"]))]
                postspawn_node = Node(package=ros2_node["package"],
                    executable=ros2_node["executable"],
                    name=str(ros2_node["name"]), 
                    output=ros2_node["output"],
                    parameters=ros2_node["parameters"],
                    remappings=remappings)
            else:
                postspawn_node = Node(package=ros2_node["package"],
                    executable=ros2_node["executable"],
                    name=str(ros2_node["name"]), 
                    output=ros2_node["output"],
                    parameters=ros2_node["parameters"])

            ld.add_action(prespawn_node)

    ####################################################################################

    # Initialize models
    for model_params in models:

        generate_model_params = models[model_params]["generate_params"]
        instance = int(models[model_params]["instance"])

        # Assign unique model name if not explicitly set in JSON
        if generate_model_params["model_name"] == "NotSet":
            generate_model_params["model_name"] = 'sitl_{:s}_{:d}'.format(
                generate_model_params["base_model"], instance)

        if "controller" not in generate_model_params:
            generate_model_params["controller"] = default_controller
        
        generate_model_params["controller"] = str(generate_model_params["controller"]).upper()

        # Reset model generation args and pull new ones from JSON
        generate_model_args = ""
        for params in generate_model_params:
            generate_model_args += ' --{:s} "{:s}"'.format(
                params, str(generate_model_params[params]))

        # Model generation command using scripts/jinja_model_gen.py
        generate_model = ['python3 {:s}/{:s}/scripts/jinja_model_gen.py{:s}'.format(
            ros2_ws, models[model_params]["gazebo_name"], 
            generate_model_args).replace("\n","").replace("    ","")]

        # Calculate spawn locations
        spawn_pose = models[model_params]["spawn_pose"]
        latitude_vehicle = float(latitude) + ((float(spawn_pose[1])/6378137.0)*(180.0/np.pi))
        longitude_vehicle = float(longitude) + ((float(spawn_pose[0])/
            (6378137.0*np.cos((np.pi*float(latitude))/180.0)))*(180.0/np.pi))
        altitude_vehicle = float(altitude) + float(spawn_pose[2])

        # Execute jinja generator
        jinja_model_generate = ExecuteProcess(
            cmd=generate_model,
            name='jinja_gen_{:s}'.format(generate_model_params["model_name"]),
            shell=True,
            output='screen')

        ld.add_action(jinja_model_generate)


        if str(generate_model_params["controller"]) == "PX4":
            LogInfo(msg='\nINFO: Model: %s using \"controller\": \"%s\".\n' % 
                (str(generate_model_params["model_name"]),str(generate_model_params["controller"])))
            print('\nINFO: Model: {:s} using \"controller\": \"{:s}\".\n'.format(
                str(generate_model_params["model_name"]),str(generate_model_params["controller"])))

            
            # See if rtps_args exist for model
            if "rtps_agent_args" in models[model_params]:

                # Command to run micrortps agent based on instance if present but "NotSet"
                # This works in conjunction with the ROMFS/px4fmu_common/init.d-posix/airframes/<ID>_<MODEL>.post
                if (models[model_params]["rtps_agent_args"] == "NotSet") and ("namespace" in generate_model_params):
                    urtps_agent_cmd = '''eval \"micrortps_agent -n {:s} -t UDP -r {:s} -s {:s}\"; 
                        bash'''.format(str(generate_model_params["namespace"]),
                        str(2020+(2*instance)), str(2019+(2*instance))
                            ).replace("\n","").replace("    ","")

                elif (models[model_params]["rtps_agent_args"] == "NotSet") and ("namespace" not in generate_model_params):
                    urtps_agent_cmd = '''eval \"micrortps_agent -t UDP -r {:s} -s {:s}\"; 
                        bash'''.format(str(2020+(2*instance)), str(2019+(2*instance))
                            ).replace("\n","").replace("    ","")

                # Command to run micrortps agent with assigned args
                else:
                    urtps_agent_cmd = '''eval \"micrortps_agent {:s}\"; 
                        bash'''.format(models[model_params]["rtps_agent_args"]
                            ).replace("\n","").replace("    ","")

                # Xterm command to name xterm window and run urtps_agent_cmd
                xterm_urtps_agent_cmd = ['''xterm -hold -T \"{:s}\" 
                    -n \"{:s}\" -e \'{:s}\''''.format("micrortps_agent", 
                        "micrortps_agent", urtps_agent_cmd
                        ).replace("\n","").replace("    ","")]
                
                # Run agent command
                micrortps_agent = ExecuteProcess(
                    cmd=xterm_urtps_agent_cmd,
                    name='xterm_urtps_agent_{:s}'.format(generate_model_params["model_name"]),
                    shell=True)
        
                ld.add_action(micrortps_agent)

            # Path for PX4 binary storage
            sitl_output_path = '/tmp/{:s}'.format(generate_model_params["model_name"])

            # Command to make storage folder
            sitl_folder_cmd = ['mkdir -p \"{:s}\"'.format(sitl_output_path)]

            
            # Set each xterm with PX4 environment variables
            px4_env = '''export PX4_SIM_MODEL=\"{:s}\"; export PX4_HOME_LAT={:s}; 
                export PX4_HOME_LON={:s}; export PX4_HOME_ALT={:s};'''.format(
                    generate_model_params["base_model"], str(latitude_vehicle), 
                    str(longitude_vehicle), str(altitude_vehicle)
                    ).replace("\n","").replace("    ","")

            # Set path for PX4 build
            px4_path = '{:s}/{:s}/build/{:s}'.format(ros2_ws,
                models[model_params]["autopilot_name"],
                models[model_params]["autopilot_build_type"])

            # Command to export model and run PX4 binary         
            px4_cmd = '''{:s} eval \"\"{:s}/bin/px4\" 
                -w {:s} \"{:s}/etc\" -s etc/init.d-posix/rcS -i {:d}\"; bash'''.format(
                    px4_env, px4_path, sitl_output_path, px4_path, instance)

            # Xterm command to name xterm window and run px4_cmd
            xterm_px4_cmd = ['''xterm -hold -T \"PX4 NSH {:s}\" 
                -n \"PX4 NSH {:s}\" -e \'{:s}\''''.format(
                    sitl_output_path, sitl_output_path,
                    px4_cmd).replace("\n","").replace("    ","")]

            # Make storage command
            make_sitl_folder = ExecuteProcess(
                cmd=sitl_folder_cmd,
                name='make_sitl_folder_{:s}'.format(generate_model_params["model_name"]),
                shell=True)
        
            ld.add_action(make_sitl_folder)

            # Run PX4 binary
            px4_posix = ExecuteProcess(
                cmd=xterm_px4_cmd,
                name='xterm_px4_nsh_{:s}'.format(generate_model_params["model_name"]),
                shell=True)
        
            ld.add_action(px4_posix)

        elif str(generate_model_params["controller"]) == "COGNIPILOT":
            LogInfo(msg='\nINFO: Model: %s using \"controller\": \"%s\".\n' % 
                (str(generate_model_params["model_name"]),str(generate_model_params["controller"])))
            print('\nINFO: Model: {:s} using \"controller\": \"{:s}\".\n'.format(
                str(generate_model_params["model_name"]),str(generate_model_params["controller"])))

        elif str(generate_model_params["controller"]) == "ROS2":
            LogInfo(msg='\nINFO: Model: %s using \"controller\": \"%s\".\n' % 
                (str(generate_model_params["model_name"]),str(generate_model_params["controller"])))
            print('\nINFO: Model: {:s} using \"controller\": \"{:s}\".\n'.format(
                str(generate_model_params["model_name"]),str(generate_model_params["controller"])))

        else:
            LogInfo(msg='\nWARN: \"controller\": \"%s\" not matched to accepted keys [\"PX4\", \"COGNIPILOT\", \"ROS2\"] in \"%s\".\n' %
                (str(generate_model_params["controller"]),str(model_params)))
            print('''\nWARN: \"controller\": \"{:s}\" not matched to accepted keys: 
                [\"PX4\", \"COGNIPILOT\", \"ROS2\"] in \"{:s}\".\n'''.format(
                    str(generate_model_params["controller"]),str(model_params)))            

        if not overide_gazebo:
            # Spawn model in gazebo
            spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', '{:s}'.format(generate_model_params["model_name"]),
                                '-timeout', str(5.0), '-spawn_service_timeout', str(3.0), '-unpause',
                                '-x', str(spawn_pose[0]), '-y', str(spawn_pose[1]), '-z', str(spawn_pose[2]),
                                '-R', str(spawn_pose[3]), '-P', str(spawn_pose[4]), '-Y', str(spawn_pose[5]),
                                '-file', '/tmp/gazebo/models/{:s}/{:s}.sdf'.format(
                                    generate_model_params["model_name"], generate_model_params["model_name"])],
                            name='spawn_{:s}'.format(generate_model_params["model_name"]), output='screen')

            ld.add_action(spawn_entity)

        if overide_gazebo:
            spawn_cmd = '''gz model --spawn-file=/tmp/gazebo/models/{:s}/{:s}.sdf --model-name={:s} 
                -x {:s} -y {:s} -z {:s} -R {:s} -P {:s} -Y {:s}'''.format(
                generate_model_params["model_name"], generate_model_params["model_name"],
                generate_model_params["model_name"], str(spawn_pose[0]),str(spawn_pose[1]),
                str(spawn_pose[2]), str(spawn_pose[3]), str(spawn_pose[4]), str(spawn_pose[5])
                ).replace("\n","").replace("    ","")
            spawn_cmd_popen=shlex.split(spawn_cmd)
            spawn_popen = subprocess.Popen(spawn_cmd_popen, 
                stdout=subprocess.PIPE, text=True)

    ####################################################################################

    # post-spawn ROS2 Nodes
    for node in ros2_nodes:
        ros2_node = ros2_nodes[node]

        # Run if timing set to post-spawn of model
        if ros2_node["timing"] == "post-spawn":
            if "remappings" in ros2_node:
                remappings = [eval(str(ros2_node["remappings"]))]
                postspawn_node = Node(package=ros2_node["package"],
                    executable=ros2_node["executable"],
                    name=str(ros2_node["name"]), 
                    output=ros2_node["output"],
                    parameters=ros2_node["parameters"],
                    remappings=remappings)
            else:
                postspawn_node = Node(package=ros2_node["package"],
                    executable=ros2_node["executable"],
                    name=str(ros2_node["name"]), 
                    output=ros2_node["output"],
                    parameters=ros2_node["parameters"])

            ld.add_action(postspawn_node)

########################################################################################


    return ld
