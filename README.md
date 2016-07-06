# ArmorPy
A python client library for [**ARMOR**](https://github.com/EmaroLab/armor).

Allows users to easily call ARMOR service from Python code.

**Full documentation** at:

http://emarolab.github.io/armor_py_api/

Hides ROS service calls to ARMOR in utility functions whose structure is 
similar to that of OWL API. ARMOR is a powerful tool able to manage, query
and modify multiple ontologies in a safe way. It also include several macros
to perform complex manipulations on ARMOR references (manipulations that
usually require more than a few line of codes in OWL APIs, such as replacing
a specific data property value associated to an individual).

More functions are coming soon as their needed.

ArmorPy installs itself into the */devel/bin* of your workspace as you run
*catkin_make install*. It is then available to all your ROS Python packages.

If your IDE cannot find the ArmorPy sources when working on a different 
project, you have to add armor_py_api/scripts/armor_api as a source folder.

In PyCharm you can add a folder as a source in the menu:

**Settings...-> Project: your_project_name-> Project Structure**.
