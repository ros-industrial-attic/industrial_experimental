

from __future__ import print_function
import em
from utils import *

"""
BasePackageGenerator

Base class interface to a package generator
"""
class BasePackageGenerator():

    
  def _generate_empy_file(self, em_params, template_paths, template_name, file_path, file_name):
    print("Generating " + file_name)
    
    template_file = None
    for template_path in template_paths:
      template_file = template_path + "/" + template_name
      try:
        f = open(template_file, 'r')
        #quit loop on first successful open
        break  
      except IOError:
        pass
    print("Loading template file: " + template_file)
        
    text = em.expand(f.read(), em_params)
    package_file = file_path + "/" + file_name
    
    if(text):
      print("Outputting package file: " + package_file)
      fd = open(package_file, 'w')
      fd.write(text)
    else:
      print("Generated package file: " + package_file + ", contents empty, no file created")
    
    
    
"""
SupportPackageGenerator

This class encapsulates functionality to generate standard ROS-I support 
packages.  This class uses templates (em.py) to generate package files.
Templates can be changed to alter the generated files.
"""
class SupportPackageGenerator(BasePackageGenerator):

  def __init__(self):
    pass
    
  def generate_package(self, prefix, model, num_joints, author, author_email, version, template_paths):
    print("Generating support directories for model: ", model)
    
    #Handling special parameters
    if prefix:
      package = prefix + '_' + model + '_support'
    else:
      package = model + '_support'
      
    #Load arguments into em params
    em_params = self._load_em_params(package, model, num_joints, author, author_email, version, prefix)
        
    #Populate package root directory
    file_path = package
    mkdir(file_path)
    self._generate_cmakelist(em_params, template_paths, file_path)
    self._generate_package_xml(em_params, template_paths, file_path)
    
    #Populate package/config directory
    file_path = package + "/config"
    mkdir(file_path)
    self._generate_joint_names_yaml(em_params, template_paths, file_path, model)

    #Populate package/launch directory
    file_path = package + "/launch"
    mkdir(file_path)
    self._generate_load_launch(em_params, template_paths, file_path, model)
    self._generate_interface_launch(em_params, template_paths, file_path, model)
    self._generate_visualize_launch(em_params, template_paths, file_path, model)
    self._generate_test_model_launch(em_params, template_paths, file_path, model)
    #Populate package/meshes directory
    mkdir(package + "/meshes")
    mkdir(package + "/meshes/" + model)
    mkdir(package + "/meshes/" + model + "/collision")
    mkdir(package + "/meshes/" + model + "/visual")
    #Populate package/test directory
    file_path = package + "/test"
    mkdir(file_path)
    self._generate_launch_test(em_params, template_paths, file_path)
    mkdir(package + "/urdf")
    touch(package + "/urdf/" + model + ".urdf")
    touch(package + "/urdf/" + model + ".xacro")
    touch(package + "/urdf/" + model + "_macro.xacro")

    print("TODO: ADD COLLISION/VISUAL MESHES")
    print("TODO: FILL IN URDF MACROS/URDFS")
    
  def _load_em_params(self, package, model, num_joints, author, author_email, version, prefix):
    
    #Load empy parameters
    em_params = {}
    em.expand('@{package = "' + package + '"}', em_params)
    em.expand('@{model = "' + model + '"}', em_params)
    em.expand('@{num_joints = "' + str(num_joints) + '"}', em_params)
    em.expand('@{author = "' + author + '"}', em_params)
    em.expand('@{email = "' + author_email + '"}', em_params)
    em.expand('@{pkg_vers = "' + version + '"}', em_params)
    em.expand('@{prefix = "' + prefix + '"}', em_params)
    
    return em_params
    
  def _generate_cmakelist(self, em_params, template_paths, file_path):
    self._generate_empy_file(em_params, template_paths, "CMakeLists.empy", file_path, "CMakeLists.txt")
    
  def _generate_package_xml(self, em_params, template_paths, file_path):
    self._generate_empy_file(em_params, template_paths, "package.empy", file_path, "package.xml")
    
  def _generate_joint_names_yaml(self, em_params, template_paths, file_path, model):
    file_name = "joint_names_" + model + ".yaml"
    self._generate_empy_file(em_params, template_paths, "joint_names.empy", file_path, file_name)

  def _generate_load_launch(self, em_params, template_paths, file_path, model):
    file_name = "load_" + model  + ".launch"
    self._generate_empy_file(em_params, template_paths, "load_launch.empy", file_path, file_name)
    
  def _generate_interface_launch(self, em_params, template_paths, file_path, model):
    file_name = "robot_interface_streaming_" + model + ".launch"
    self._generate_empy_file(em_params, template_paths, "streaming_interface.empy", file_path, file_name)
    file_name = "robot_interface_download_" + model + ".launch"
    self._generate_empy_file(em_params, template_paths, "download_interface.empy", file_path, file_name)

  def _generate_visualize_launch(self, em_params, template_paths, file_path, model):
    file_name = "robot_state_visualize_" + model  + ".launch"
    self._generate_empy_file(em_params, template_paths, "robot_state_visualize.empy", file_path, file_name)

  def _generate_test_model_launch(self, em_params, template_paths, file_path, model):
    file_name = "test_" + model  + ".launch"
    self._generate_empy_file(em_params, template_paths, "test_model.empy", file_path, file_name)

  def _generate_launch_test(self, em_params, template_paths, file_path):
    self._generate_empy_file(em_params, template_paths, "roslaunch_test.empy", file_path, "roslaunch_test.xml")

    
    
    
    
    
"""
MoveitPackageGenerator

This class encapsulates functionality to generate standard ROS-I MoveIt 
packages.  This class uses templates (em.py) to generate package files.
Templates can be changed to alter the generated files.
"""
class MoveitPackageGenerator(BasePackageGenerator):

  def __init__(self):
    pass
    
  def generate_package(self, prefix, model, num_joints, assistant, template_paths):
    print("Generating MoveIt directories for model: ", model)
    
    #Handling special parameters
    if prefix:
      moveit_package = prefix + '_' + model + '_moveit_config'
      support_package = prefix + '_' + model + '_support'
    else:
      moveit_package = model + '_moveit_config'
      support_package = model + '_support'
      
      
    #Load arguments into em params
    em_params = self._load_em_params(moveit_package, support_package, model, num_joints)
    
    #Start setup assistant
    if assistant:
      self._setup_assistant()
    
    #Create package directory structure (if it doesn't exist)
    mkdir(moveit_package)
    file_path = moveit_package + "/launch"
    mkdir(file_path)
    self._generate_controller_manager_launch(em_params, prefix, template_paths, file_path, model)
    self._generate_planning_execution_launch(em_params, template_paths, file_path)
    
    file_path = moveit_package + "/config"
    mkdir(file_path)
    self._generate_controllers_yaml(em_params, template_paths, file_path)
    
    print("TODO: CHECK JOINT LIMITS YAML FOR ACCEL/DECEL LIMITS")
        
    
  def _load_em_params(self, moveit_package, support_package, model, num_joints):
    
    #Load empy parameters
    em_params = {}
    em.expand('@{moveit_package = "' + moveit_package + '"}', em_params)
    em.expand('@{support_package = "' + support_package + '"}', em_params)
    em.expand('@{model = "' + model + '"}', em_params)
    em.expand('@{num_joints = "' + str(num_joints) + '"}', em_params)
    
    return em_params
  
  def _generate_controller_manager_launch(self, em_params, prefix, template_paths, file_path, model):
    if prefix:
      file_name = prefix + "_" + model  + "_moveit_controller_manager.launch.xml"
    else:
      file_name = model  + "_moveit_controller_manager.launch.xml"
    self._generate_empy_file(em_params, template_paths, "controller_manager.empy", file_path, file_name)
    
  def _generate_planning_execution_launch(self, em_params, template_paths, file_path):
    self._generate_empy_file(em_params, template_paths, "moveit_planning_execution.empy", file_path, "moveit_planning_execution.launch")
    
  
  def _generate_controllers_yaml(self, em_params, template_paths, file_path):
    self._generate_empy_file(em_params, template_paths, "controllers.empy", file_path, "controllers.yaml")
    
  def _setup_assistant(self):
    print("The following will open a webpage with instructions on how to create a MoveIt package as well as initiate the MoveIt setup assistant.")
    raw_input("Press any key to continue:")
    os.system("gnome-open http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot")
    os.system("roslaunch moveit_setup_assistant setup_assistant.launch")


