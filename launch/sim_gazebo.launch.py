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


launch_path = os.path.realpath(__file__).replace("sim_gazebo.launch.py","")
json_path = os.path.realpath(os.path.relpath(os.path.join(launch_path,"../config")))
ros2_ws = os.path.realpath(os.path.relpath(os.path.join(launch_path,"../../..")))
gazebo_model_reset_env=False
gazebo_plugin_reset_env=False


with open('{:s}/gen_params.json'.format(json_path)) as json_file:
    json_params = json.load(json_file)

setup_autopilot = json_params["setup"]["autopilot"]
setup_gazebo = json_params["setup"]["gazebo"]

models = json_params["models"]
world_params = json_params["world_params"]
generate_world_params = world_params["generate_params"]


for repo in setup_gazebo["gazebo_models"]:
    gazebo_repo = setup_gazebo["gazebo_models"][repo]
    gazebo_repo_path = '{:s}/{:s}'.format(ros2_ws, 
        gazebo_repo["name"])
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
    if not gazebo_model_reset_env:
        os.environ['GAZEBO_MODEL_PATH'] = '{:s}/models'.format(
            gazebo_repo_path)
        gazebo_model_reset_env=True
    elif '{:s}/models'.format(gazebo_repo_path) not in os.getenv('GAZEBO_MODEL_PATH'):
        os.environ['GAZEBO_MODEL_PATH'] = '{:s}/models:{:s}'.format(
            gazebo_repo_path, os.getenv('GAZEBO_MODEL_PATH'))

for build in setup_autopilot:
    autopilot_build = setup_autopilot[build]
    autopilot_path = '{:s}/{:s}'.format(ros2_ws, 
        autopilot_build["name"])
    autopilot_build_path = '{:s}/build/{:s}'.format(autopilot_path, 
        autopilot_build["build_type"])
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

    if os.getenv('LD_LIBRARY_PATH') is None:
         os.environ['LD_LIBRARY_PATH'] = '{:s}/build_gazebo'.format(
            autopilot_build_path)

    elif '{:s}/build_gazebo'.format(autopilot_build_path) not in os.getenv('LD_LIBRARY_PATH'):
        os.environ['LD_LIBRARY_PATH'] = '{:s}/build_gazebo:{:s}'.format(
            autopilot_build_path, os.getenv('LD_LIBRARY_PATH'))

    if not gazebo_plugin_reset_env and autopilot_build["gazebo_plugins"]:
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}/build_gazebo'.format(
            autopilot_build_path)
        gazebo_plugin_reset_env=True

    elif autopilot_build["gazebo_plugins"] and ('{:s}/build_gazebo'.format(autopilot_build_path) 
        not in os.getenv('GAZEBO_PLUGIN_PATH')):
        os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}/build_gazebo:{:s}'.format(
            autopilot_build_path, os.getenv('GAZEBO_PLUGIN_PATH'))

if os.getenv('GAZEBO_PLUGIN_PATH') is None:
    os.environ['GAZEBO_PLUGIN_PATH'] = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"

elif "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" not in os.getenv('GAZEBO_PLUGIN_PATH'):
    os.environ['GAZEBO_PLUGIN_PATH'] = '{:s}:{:s}'.format(
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins", 
            os.getenv('GAZEBO_PLUGIN_PATH'))

os.environ['GAZEBO_RESOURCE_PATH'] = "/usr/share/gazebo-11"

if world_params["generate_world"]:
    generate_world_args=""
    for params in generate_world_params:
        generate_world_args += ' --{:s} "{:s}"'.format(params, 
            str(generate_world_params[params]))

    generate_world_cmd = 'python3 {:s}/{:s}/scripts/jinja_world_gen.py{:s}'.format(
        ros2_ws, world_params["gazebo_name"], generate_world_args
        ).replace("\n","").replace("    ","")

    world_cmd_popen=shlex.split(generate_world_cmd)
    world_popen = subprocess.Popen(world_cmd_popen, stdout=subprocess.PIPE, text=True)
    while True:
        output = world_popen.stdout.readline()
        if output == '' and world_popen.poll() is not None:
            break
        if output:
            print(output.strip())
    world_popen.wait()

    world_file_path='/tmp/{:s}.world'.format(generate_world_params["world_name"])

else:
    world_file_path='{:s}/{:s}/worlds/{:s}.world'.format(ros2_ws,
        world_params["gazebo_name"],generate_world_params["world_name"])

latitude = generate_world_params["latitude"]
longitude = generate_world_params["longitude"]
altitude = generate_world_params["altitude"]


def generate_launch_description():

    ld = LaunchDescription([
    	# World path argument
        DeclareLaunchArgument(
            'world_path', default_value= world_file_path,
            description='Provide full world file path and name'),
        LogInfo(msg=LaunchConfiguration('world_path')),
        ])

    # Get path to gazebo package
    gazebo_package_prefix = get_package_share_directory('gazebo_ros')

    # Launch gazebo servo with world file from world_path
    gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzserver.launch.py']),
                launch_arguments={'world': LaunchConfiguration('world_path')}.items(),
                )
    
    ld.add_action(gazebo_server)

    instance = 0
    for model_params in models:

        generate_model_params = models[model_params]["generate_params"]

        if generate_model_params["model_name"] == "NotSet":
            generate_model_params["model_name"] = 'sitl_{:s}_{:d}'.format(
                generate_model_params["base_model"],instance)

        # Path for PX4 binary storage
        sitl_output_path = '/tmp/{:s}'.format(generate_model_params["model_name"])

        
        generate_model_args = ""
        for params in generate_model_params:
            generate_model_args += ' --{:s} "{:s}"'.format(
                params, str(generate_model_params[params]))

        generate_model = ['python3 {:s}/{:s}/scripts/jinja_model_gen.py{:s}'.format(
            ros2_ws, models[model_params]["gazebo_name"], 
            generate_model_args).replace("\n","").replace("    ","")]

        # Command to make storage folder
        sitl_folder_cmd = ['mkdir -p \"{:s}\"'.format(sitl_output_path)]

        # Calculate spawn locations
        spawn_pose = models[model_params]["spawn_pose"]
        latitude_vehicle = float(latitude) + ((float(spawn_pose[1])/6378137.0)*(180.0/np.pi))
        longitude_vehicle = float(longitude) + ((float(spawn_pose[0])/
            (6378137.0*np.cos((np.pi*float(latitude))/180.0)))*(180.0/np.pi))
        altitude_vehicle = float(altitude) + float(spawn_pose[2])
        
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

        # Execute jinja generator
        jinja_model_generate = ExecuteProcess(
            cmd=generate_model,
            name='jinja_gen_{:s}'.format(generate_model_params["model_name"]),
            shell=True,
            output='screen')

        ld.add_action(jinja_model_generate)

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

        # Spawn entity
        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', '{:s}'.format(generate_model_params["model_name"]),
                            '-x', str(spawn_pose[0]), '-y', str(spawn_pose[1]), '-z', str(spawn_pose[2]),
                            '-R', str(spawn_pose[3]), '-P', str(spawn_pose[4]), '-Y', str(spawn_pose[5]),
                            '-file', '/tmp/{:s}.sdf'.format(generate_model_params["model_name"])],
                        name='spawn_{:s}'.format(generate_model_params["model_name"]), output='screen')

        ld.add_action(spawn_entity)
        
        # Increment instance
        instance += 1

    # Launch gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzclient.launch.py']))
    
    LogInfo(msg="\nWaiting to launch Gazebo Client...\n")
    sleep(2)

    ld.add_action(gazebo_client)

    return ld
