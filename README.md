# pyarmor
A python client library for armor.

Allows users to easily call ARMOR service from Python code. 

ROS service calls are hidden in utility functions whose structure is similar to that of OWLAPI or OWLAPI helper functions, thus hiding completely the fact that the fact that the ontology is remotely managed by ARMOR instead of your node itself. It also include several macros to perform complex manipulations on ARMOR references (manipulations that require a more than a single call to ARMOR, such as replacing a specific data property value associated to specific individual).

More functions are coming soon as their needed.

Pyarmor installs itself into the /devel/bin of your workspace as you run catkin_make. It is then available to all your ROS Python packages. At the moment IDEs do not recognize it, but it will work at runtime. Any help on this side will be most welcome.
