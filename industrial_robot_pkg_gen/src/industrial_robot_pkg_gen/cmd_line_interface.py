
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
  def __init__(self):
    
    self.parser = argparse.ArgumentParser(description='Creates ROS-Industrial vendor robot package')
    self.parser.add_argument('--t_pkg', default= 'industrial_robot_pkg_gen', help='package that contains templates')
    self.parser.add_argument('--t_path', default= 'resources', help='templates(*.empy) path, relative to package specified by t_pkg argument')
    self.parser.add_argument('--num_joints', type=int, default=6, help='number of robot dof(joints)')
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
  
  def __init__(self, subparser):
    pass
    
  def add_to_subparser(self, subparser):
    pass
    
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
    parser_support.add_argument('--prefix', default= None, help='prefix to be added to package name (typically meta-package name)')
    parser_support.set_defaults(func=self._execute)
    
  def _execute(self, args):
    print(args)

    # allows us to get the package path
    rospack = rospkg.RosPack()
    template_path = rospack.get_path(args.t_pkg) + '/' + args.t_path
    print("Template path: " + template_path)
    
    # Author email used in place of empty author name
    if(not args.author): args.author = args.email
    
    generator = SupportPackageGenerator()
    generator.generate_package(args.prefix, args.model, args.num_joints, args.author, args.email, args.pkg_vers, template_path)

    

"""
MoveitSubCmd
"""
class MoveitSubCmd(SubCmdBase):

  def __init__(self):
    self.name = 'moveit'
    
  def add_to_subparser(self, subparser):
    parser = subparser.add_parser(self.name, description='Creates ROS-Industrial support package')
    parser.add_argument('model', help='robot model number')
    parser.add_argument('--prefix', default= None, help='prefix to be added to package name (typically meta-package name)')
    parser.add_argument('--setup', default=True, help='True by default.  If true, moveit setup assistant is called by script')
    parser.set_defaults(func=self._execute)
    
  def _execute(self, args):
    print(args)

    # allows us to get the package path
    rospack = rospkg.RosPack()
    template_path = rospack.get_path(args.t_pkg) + '/' + args.t_path
    print("Template path: " + template_path)
        
    generator = MoveitPackageGenerator()
    generator.generate_package(args.prefix, args.model, args.num_joints, args.setup, template_path)

