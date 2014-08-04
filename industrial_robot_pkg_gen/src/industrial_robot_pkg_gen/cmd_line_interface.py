
from __future__ import print_function
import os      
import argparse
import rospkg

from generators import *
     
"""
CmdLineInterface

This class encapsulates the package generator command line interface (CLI).  
Sub-commands are added via the add_sub_cmd method.  This allows the class
to dynamically add commands for package generation (including specialized
commands for specific packages).
"""  
class CmdLineInterface:


  """
  Constructor command line interface object
  
  """
  def __init__(self, def_t_paths = ['industrial_robot_pkg_gen/resources'], def_num_joints = 6, def_prefix = None):
    
    self.parser = argparse.ArgumentParser(description='Creates ROS-Industrial vendor robot package')
    self.parser.add_argument('--t_paths', nargs='*', default = def_t_paths, help='templates(<package>/<local_path> path(s)(in order), default:=' + str(def_t_paths) )
    self.parser.add_argument('--prefix', default= def_prefix, help='prefix to be added to package name (typically meta-package name), default:=' + str(def_prefix) )
    self.parser.add_argument('--num_joints', type=int, default = def_num_joints, help='number of robot dof(joints), default:=' + str(def_num_joints) )
    self.subparsers = self.parser.add_subparsers()    

  def add_sub_cmd(self, sub_cmd):
    sub_cmd.add_to_subparser(self.subparsers)
 
  def run(self):
    
    args = self.parser.parse_args()
    args.func(args)



"""
SubCmdBase

This base class is an interface definition for sub commands entered on the
command line.  This class encapsulates parsing and execution methods.
"""
class SubCmdBase:
  
  def __init__(self):
    pass
    
  def add_to_subparser(self, subparser):
    pass
    
  def _eval_t_paths(self, t_paths):
    
    template_paths = []
    for template_path in t_paths:
      template_path_split = template_path.split('/', 1)
      package = template_path_split[0]
      path = template_path_split[1]
      rospack = rospkg.RosPack()
      full_path = rospack.get_path(package) + '/' + path
      template_paths.append(full_path)
    print("Template path(s): " + str(template_paths) )
    return template_paths
    
  def _execute(self):
    pass
    
    
"""
SupportSubCmd
"""
class SupportSubCmd(SubCmdBase):

  def __init__(self):
    self.name = 'support'
    
  def add_to_subparser(self, subparser):
    parser_support = subparser.add_parser(self.name, description='Creates ROS-Industrial support package')
    parser_support.add_argument('model', help='robot model number')
    parser_support.add_argument('email', help='author email')
    parser_support.add_argument('--pkg_vers', default= '0.0.1', help='package version number')
    parser_support.add_argument('--author', default= None, help='author name')
    parser_support.set_defaults(func=self._execute)
    
  def _execute(self, args):
    print(args)
    
    template_paths = self._eval_t_paths(args.t_paths)
    
    # Author email used in place of empty author name
    if(not args.author): args.author = args.email
    
    generator = SupportPackageGenerator()
    generator.generate_package(args.prefix, args.model, args.num_joints, args.author, args.email, args.pkg_vers, template_paths)

    

"""
MoveitSubCmd
"""
class MoveitSubCmd(SubCmdBase):

  def __init__(self):
    self.name = 'moveit'
    
  def add_to_subparser(self, subparser):
    parser = subparser.add_parser(self.name, description='Creates ROS-Industrial support package')
    parser.add_argument('model', help='robot model number')
    parser.add_argument('--setup', default=True, help='True by default.  If true, moveit setup assistant is called by script')
    parser.set_defaults(func=self._execute)
    
  def _execute(self, args):
    print(args)

    template_paths = self._eval_t_paths(args.t_paths)
        
    generator = MoveitPackageGenerator()
    generator.generate_package(args.prefix, args.model, args.num_joints, args.setup, template_paths)

